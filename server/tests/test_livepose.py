"""Tests for vacuula_server.livepose (live robot position).

    cd server && uv run --with pytest pytest tests

(the glue tests need the server dependencies and skip themselves without sqlalchemy)
"""
import json
import math
import socket
import struct
import threading
import time

import pytest

from vacuula_server import livepose as live
from vacuula_server import livepose_core as lp

SERIAL = "GPC00000-0012a0b0c0d0"
IP = "192.0.2.50"
MAP_ID = "2020-01-01T00:00:00Z"


def packet(x_m, y_m, theta, flags=0, ver=4):
    x = int(round(x_m / lp.TIC_M * lp.FIX))
    y = int(round(y_m / lp.TIC_M * lp.FIX))
    th = int(round((theta % (2 * math.pi)) / lp.BAM)) & 0xFFFFFFFF
    th = th - (1 << 32) if th >= (1 << 31) else th
    return struct.pack(lp.BASE_FMT, lp.NEAP_MAGIC, ver, flags, 0, 0, x, y, th, 0, 0, 0xFF, 0)


class FixedCalibration:
    """Stands in for livepose.json: one map with robot (x, y) m -> px (260 + 24.1 x, 130 - 24.1 y)."""

    def get(self, now=None):
        return {"maps": {MAP_ID: {"robot_to_px": [[24.1, 0.0, 260.0], [0.0, -24.1, 130.0]]}}}


class Rig:
    """A hub with fake clocks that records what would go to the web clients."""

    def __init__(self, calibration=None, dock_pixel=None):
        self.events = []
        self.hub = live.PoseHub(lambda serial, payload: self.events.append((serial, payload)),
                                lambda ip: SERIAL if ip == IP else None, calibration or FixedCalibration(), dock_pixel)
        self.t = 100.0

    def send(self, x, y, theta=0.0, dt=1 / 18, ip=IP, **kw):
        self.t += dt
        self.hub.handle(packet(x, y, theta, **kw), ip, now=self.t, wall=1_800_000_000 + self.t)

    def tick(self, advance=0.0):
        self.t += advance
        self.hub.tick(now=self.t, wall=1_800_000_000 + self.t)

    def kinds(self):
        return [p["kind"] for _, p in self.events]

    def clean(self, seconds=6.0):
        """One clean: dock frame, two localization jumps 0.4 s apart, then a drive along +x at 0.2 m/s from
        (0.05, 0.02) in the map frame (never exactly (0, 0, 0): that means docked). Returns the final x."""
        for _ in range(18):
            self.send(0.284, 0.0)
        self.send(1.0, 0.5, 1.0)
        self.send(0.05, 0.02, 0.3, dt=0.4)
        for _ in range(20):
            self.send(0.05, 0.02, 0.3)       # >= 1 s after the last jump: localized
        x = 0.05
        for _ in range(int(seconds * 18)):
            x += 0.2 / 18
            self.send(x, 0.02, 0.3)
        return x


@pytest.fixture
def rig():
    return Rig()


# ---------------------------------------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------------------------------------

class TestCalibration:
    def test_no_calibration_without_file(self, tmp_path):
        cal = live.Calibration(tmp_path / "livepose.json")
        assert cal.get(0.0)["maps"] == {}

    def test_file_is_loaded_and_reloads_on_change(self, tmp_path):
        path = tmp_path / "livepose.json"
        path.write_text(json.dumps({"maps": {"m1": {"robot_to_px": [[1, 2, 3], [4, 5, 6]]}}}))
        cal = live.Calibration(path, check_every_s=0)
        assert cal.get(0.0)["maps"] == {"m1": {"robot_to_px": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]}}
        path.write_text(json.dumps({"maps": {"m2": {"robot_to_px": [[1, 0, 0], [0, 1, 0]]}}}))
        import os
        os.utime(path, ns=(1, 2_000_000_000))       # make the mtime differ whatever the fs timestamp resolution
        assert list(cal.get(1.0)["maps"]) == ["m2"]

    def test_checks_the_file_at_most_once_per_interval(self, tmp_path):
        path = tmp_path / "livepose.json"
        path.write_text(json.dumps({"maps": {"m1": {"robot_to_px": [[1, 0, 0], [0, 1, 0]]}}}))
        cal = live.Calibration(path, check_every_s=10)
        assert list(cal.get(0.0)["maps"]) == ["m1"]
        path.write_text(json.dumps({"maps": {"m2": {"robot_to_px": [[1, 0, 0], [0, 1, 0]]}}}))
        import os
        os.utime(path, ns=(1, 3_000_000_000))
        assert list(cal.get(5.0)["maps"]) == ["m1"]
        assert list(cal.get(11.0)["maps"]) == ["m2"]

    def test_broken_file_keeps_previous(self, tmp_path):
        path = tmp_path / "livepose.json"
        path.write_text(json.dumps({"maps": {"m1": {"robot_to_px": [[1, 0, 0], [0, 1, 0]]}}}))
        cal = live.Calibration(path, check_every_s=0)
        assert list(cal.get(0.0)["maps"]) == ["m1"]
        path.write_text("{not json")
        import os
        os.utime(path, ns=(1, 4_000_000_000))
        assert list(cal.get(1.0)["maps"]) == ["m1"]

    def test_invalid_entries_are_skipped(self):
        maps = live.parse_calibration({"maps": {
            "ok": {"robot_to_px": [[1, 2, 3], [4, 5, 6]]},
            "short": {"robot_to_px": [[1, 2, 3]]},
            "text": {"robot_to_px": [[1, 2, "x"], [4, 5, 6]]},
            "nan": {"robot_to_px": [[1, 2, float("nan")], [4, 5, 6]]},
            "none": {},
        }})
        assert list(maps) == ["ok"]
        with pytest.raises(ValueError):
            live.parse_calibration([])


# ---------------------------------------------------------------------------------------------------------
# Hub
# ---------------------------------------------------------------------------------------------------------

class TestHub:
    def test_clean_sequence(self, rig):
        rig.clean()
        kinds = rig.kinds()
        assert kinds[0] == "localizing"
        assert set(kinds[1:]) == {"pose"} and len(kinds) > 5

    def test_clean_then_reset(self, rig):
        end = rig.clean()
        for _ in range(40):
            rig.send(0, 0, 0)            # reset: exact zero pose, packets keep coming
        kinds = rig.kinds()
        assert kinds.count("localizing") == 1
        assert kinds.count("reset") == 1 and kinds[-1] == "reset"
        snap = rig.hub.snapshot(SERIAL, now=rig.t, wall=1_800_000_000 + rig.t)
        assert snap["status"] == "reset"
        assert snap["pose"]["x"] == pytest.approx(end, abs=0.02)      # last pose is kept for the dock phase
        rig.tick(30)                      # a reset (docked) robot is not stale
        assert rig.kinds().count("stale") == 0

    def test_event_payloads(self, rig):
        rig.clean()
        serial, loc = rig.events[0]
        assert serial == SERIAL
        assert loc["kind"] == "localizing" and loc["serial"] == SERIAL and isinstance(loc["ts"], float)
        assert "x" not in loc
        pose = next(p for _, p in rig.events if p["kind"] == "pose")
        assert set(pose) == {"serial", "kind", "ts", "x", "y", "theta", "segment", "clean"}
        assert pose["clean"] == 1
        json.dumps(pose)

    def test_pose_events_are_throttled(self, rig):
        rig.clean(seconds=10)
        poses = [p for _, p in rig.events if p["kind"] == "pose"]
        driving = [p for p in poses if p["x"] > 0.05]
        gaps = [b["ts"] - a["ts"] for a, b in zip(driving, driving[1:])]
        assert gaps and min(gaps) >= live.PUBLISH_MIN_INTERVAL_S - 0.001
        assert len(driving) <= 10 * 5 + 2        # <= 5 per second, against 18 packets per second

    def test_stale_once_then_pose_resumes_immediately(self, rig):
        rig.clean()
        n = len(rig.events)
        rig.tick(1.0)
        assert len(rig.events) == n                                    # 1 s of silence is not stale yet
        rig.tick(3.0)
        assert rig.kinds()[-1] == "stale"
        rig.tick(5.0)
        rig.tick(5.0)
        assert rig.kinds().count("stale") == 1
        snap = rig.hub.snapshot(SERIAL, now=rig.t, wall=1_800_000_000 + rig.t)
        assert snap["status"] == "stale" and snap["pose"] is not None
        x = snap["pose"]["x"]
        rig.send(x, 0.02, 0.3)            # same spot: the throttle alone would swallow it
        assert rig.kinds()[-1] == "pose"
        assert rig.hub.snapshot(SERIAL, now=rig.t, wall=0)["status"] == "pose"

    def test_snapshot_reports_stale_before_the_tick_ran(self, rig):
        rig.clean()
        assert rig.hub.snapshot(SERIAL, now=rig.t + 5, wall=0)["status"] == "stale"

    def test_snapshot_without_data(self, rig):
        snap = rig.hub.snapshot(SERIAL, now=0.0, wall=1234.5)
        assert snap == {"serial": SERIAL, "status": "none", "ts": None, "now": 1234.5, "pose": None, "trail": [],
                        "calibration": rig.hub.calibration.get(0.0)}
        assert snap["calibration"]["maps"]

    def test_snapshot_trail_and_pose(self, rig):
        end = rig.clean()
        snap = rig.hub.snapshot(SERIAL, now=rig.t, wall=1_800_000_000 + rig.t)
        assert snap["status"] == "pose"
        assert set(snap["pose"]) == {"x", "y", "theta", "segment", "clean"}
        assert snap["pose"]["x"] == pytest.approx(end, abs=1e-3)
        assert len(snap["trail"]) >= 1 and all(len(p) == 2 for seg in snap["trail"] for p in seg)
        pts = [p for seg in snap["trail"] for p in seg]
        assert pts[-1][0] == pytest.approx(end, abs=0.04)
        json.dumps(snap)

    def test_trail_restarts_with_the_next_clean(self, rig):
        rig.clean()
        for _ in range(20):
            rig.send(0, 0, 0)
        assert len(rig.hub.snapshot(SERIAL, now=rig.t, wall=0)["trail"]) == 1   # still there after docking
        rig.clean()
        snap = rig.hub.snapshot(SERIAL, now=rig.t, wall=0)
        assert len(snap["trail"]) == 1                                   # the old clean's segment is gone
        assert snap["trail"][0][0][0] == pytest.approx(0.05, abs=0.02)
        assert rig.kinds().count("localizing") == 2

    def test_last_pose_keeps_its_own_clean_while_the_next_one_localizes(self, rig):
        rig.clean()
        for _ in range(10):
            rig.send(0, 0, 0)
        for _ in range(5):
            rig.send(0.284, 0.0)                 # next clean, still in the dock frame
        snap = rig.hub.snapshot(SERIAL, now=rig.t, wall=0)
        assert snap["status"] == "localizing" and snap["trail"] == []
        assert snap["pose"]["clean"] == 1       # the pose is still the old clean's

    def test_bad_packets_are_ignored(self, rig):
        rig.hub.handle(b"", IP)
        rig.hub.handle(b"garbage" * 20, IP)
        rig.hub.handle(packet(1, 1, 0, ver=3), IP)          # v3 does not carry the smooth pose
        rig.hub.handle(packet(1, 1, 0, flags=8), IP, now=1.0, wall=1.0)   # torn
        assert rig.events == []
        assert rig.hub.snapshot(SERIAL)["status"] == "none"

    def test_unknown_sender_is_dropped(self, rig):
        rig.send(0.284, 0.0, ip="10.9.9.9")
        assert rig.events == [] and rig.hub._tracks == {}

    def test_long_silence_starts_the_stream_over(self, rig):
        rig.clean()
        rig.t += live.RESTART_AFTER_S + 1
        rig.tick()
        n = len(rig.events)
        rig.send(0.284, 0.0)              # a fresh clean after a robot reboot starts in the dock frame again
        assert rig.kinds()[n:] == ["localizing"]

    def test_publish_errors_do_not_escape(self):
        def boom(serial, payload):
            raise RuntimeError("queue gone")
        hub = live.PoseHub(boom, lambda ip: SERIAL)
        hub.handle(packet(0.284, 0, 0), IP, now=1.0, wall=1.0)           # must not raise

    def test_two_robots_are_independent(self):
        events = []
        hub = live.PoseHub(lambda s, p: events.append((s, p["kind"])), lambda ip: {"1.1.1.1": "A", "2.2.2.2": "B"}.get(ip))
        hub.handle(packet(0.284, 0, 0), "1.1.1.1", now=1.0, wall=1.0)
        hub.handle(packet(0.284, 0, 0), "2.2.2.2", now=1.0, wall=1.0)
        assert events == [("A", "localizing"), ("B", "localizing")]


class TestHeartbeat:
    def test_stationary_robot_still_publishes_every_few_seconds(self, rig):
        rig.clean(seconds=1)
        n = len(rig.events)
        for _ in range(int(7 * 18)):             # 7 s standing still, packets keep coming
            rig.send(1.0, 0.02, 0.3)
        beats = [p for _, p in rig.events[n:]]
        assert 2 <= len(beats) <= 5 and all(p["kind"] == "pose" for p in beats)      # one every ~2 s, not 18/s


class TestMapClean:
    def test_house_clean_is_unmapped(self, rig):
        rig.hub.set_map_clean(SERIAL, False)      # state arrives before the first packet
        rig.clean()
        assert rig.kinds() == ["unmapped"]         # once; never a pose, the frame is not the saved map's
        assert rig.hub.snapshot(SERIAL, now=rig.t, wall=0)["status"] == "unmapped"
        assert rig.hub.snapshot(SERIAL, now=rig.t, wall=0)["pose"] is None

    def test_state_flip_mid_stream(self, rig):
        for _ in range(5):
            rig.send(0.284, 0.0)
        rig.hub.set_map_clean(SERIAL, False)
        rig.send(0.284, 0.0)
        assert rig.kinds() == ["localizing", "unmapped"]
        rig.hub.set_map_clean(SERIAL, True)
        rig.send(1.0, 0.5, 1.0)
        rig.send(0.05, 0.02, 0.3, dt=0.4)
        for _ in range(20):
            rig.send(0.05, 0.02, 0.3)
        assert rig.kinds()[-1] == "pose"

    def test_unmapped_goes_stale_too(self, rig):
        rig.hub.set_map_clean(SERIAL, False)
        rig.send(0.284, 0.0)
        rig.tick(5)
        assert rig.kinds() == ["unmapped", "stale"]

    def test_note_state_feeds_the_filter(self, rig):
        busy = {"eventId": "77", "data": {"state": 2, "action": 11, "cleaning": {"category": 2}}}
        rig.hub.note_state(SERIAL, busy)           # house clean
        rig.send(0.284, 0.0)
        assert rig.kinds() == ["unmapped"]
        rig.hub.note_state(SERIAL, {"state": 1, "action": 0, "cleaning": {"category": 4}, "data": {}})   # idle
        assert rig.hub._tracks[SERIAL].filter.map_clean is None
        rig.hub.note_state(SERIAL, {"eventId": "12", "data": {"foo": 1}})        # not a state message: ignored
        assert rig.hub._tracks[SERIAL].filter.map_clean is None

    @pytest.mark.parametrize("body,expected", [
        ({"state": 2, "action": 11, "cleaning": {"category": 4}}, True),
        ({"state": 3, "action": 6, "cleaning": {"category": 4}}, True),        # paused / suspended, same clean
        ({"state": 2, "action": 1, "cleaning": {"category": 2}}, False),
        ({"state": 2, "action": 2, "cleaning": {"category": 3}}, False),
        ({"state": 2, "action": 3, "cleaning": {"category": 1}}, False),
        ({"state": 2, "action": 12, "cleaning": {"category": 4}}, False),       # exploring a new map
        ({"state": 3, "action": 15, "cleaning": {"category": 4}}, False),
        ({"state": 2, "action": 13, "cleaning": {"category": 4}}, False),       # acquiring map ids
        ({"state": 2, "action": 14, "cleaning": {"category": 4}}, False),       # creating / uploading a map
        ({"state": 1, "action": 0, "cleaning": {"category": 4}}, None),         # idle: nothing is being cleaned
        ({"state": 4, "action": 0, "cleaning": {"category": 4}}, None),         # error
        ({"state": 2, "action": 4}, None),                                       # no cleaning info
        ({"state": 2, "action": 4, "cleaning": {"category": 0}}, None),
        ({"state": 2, "action": 4, "cleaning": "?"}, None),
    ])
    def test_map_clean_from_state(self, body, expected):
        assert live.map_clean_from_state(body) is expected

    def test_state_body_shapes(self):
        inner = {"state": 2, "action": 11}
        assert live.state_body({"eventId": "77", "data": inner}) is inner      # state event
        answer = {"reqId": "5", "result": "ok", "data": {}, "state": 2, "action": 11}
        assert live.state_body(answer) is answer                                 # getRobotState answer
        assert live.state_body({"eventId": "3", "data": {"x": 1}}) is None
        assert live.state_body([1, 2]) is None and live.state_body(None) is None


class TestRestartAndDockCheck:
    def test_restart_after_silence_is_a_new_clean(self, rig):
        rig.clean()
        before = rig.hub.snapshot(SERIAL, now=rig.t, wall=0)["pose"]["clean"]
        rig.t += live.RESTART_AFTER_S + 1
        rig.send(0.284, 0.0)
        for _ in range(5):
            rig.send(0.284, 0.0)
        rig.send(1.0, 0.5, 1.0)
        rig.send(0.05, 0.02, 0.3, dt=0.4)
        for _ in range(20):
            rig.send(0.05, 0.02, 0.3)
        pose = [p for _, p in rig.events if p["kind"] == "pose"][-1]
        assert pose["clean"] == before + 1

    def dock_rig(self, px):
        # robot (0.05, 0.02) -> px (260 + 24.1*0.05, 130 - 24.1*0.02) = (261.2, 129.5) with the calibration
        return Rig(dock_pixel=lambda serial, map_id: px if map_id == MAP_ID else None)

    def run_to_reset(self, rig):
        rig.clean(seconds=0.1)
        for _ in range(5):
            rig.send(0, 0, 0)

    def test_dock_check_ok(self, caplog):
        rig = self.dock_rig((261.0, 128.0))
        with caplog.at_level("INFO", logger="vacuula_server.livepose"):
            self.run_to_reset(rig)
        assert "calibration ok" in caplog.text and "drift" not in caplog.text

    def test_dock_check_warns_when_far(self, caplog):
        rig = self.dock_rig((250.0, 100.0))
        with caplog.at_level("INFO", logger="vacuula_server.livepose"):
            self.run_to_reset(rig)
        warnings = [r for r in caplog.records if r.levelname == "WARNING"]
        assert len(warnings) == 1 and "px apart" in warnings[0].getMessage() and "drift" in warnings[0].getMessage()

    def test_dock_check_only_once_per_reset_and_never_without_pose(self, caplog):
        rig = self.dock_rig((250.0, 100.0))
        with caplog.at_level("INFO", logger="vacuula_server.livepose"):
            for _ in range(10):
                rig.send(0, 0, 0)                   # robot idle on the dock since start: no localized pose yet
        assert caplog.records == [] or all("first packet" in r.getMessage() for r in caplog.records)

    def test_read_dock_pixel(self, tmp_path, monkeypatch):
        d = tmp_path / SERIAL / "m1"
        d.mkdir(parents=True)
        (d / "floormap.json").write_text(json.dumps({"width": 403, "height": 400, "dock": {
            "pixel_x": 256.5, "pixel_y": 120.0, "normalised_x": 0.63806, "normalised_y": 0.300752}}))
        monkeypatch.setattr(live, "MAPS_DIR", tmp_path)
        assert live.read_dock_pixel(SERIAL, "m1") == (256.5, 120.0)       # raw pixel
        (d / "floormap.json").write_text(json.dumps({"width": 403, "height": 400, "dock": {"normalised_x": 0.6}}))
        assert live.read_dock_pixel(SERIAL, "m1") is None                 # no pixel coordinates: no check
        assert live.read_dock_pixel(SERIAL, "missing") is None
        assert live.read_dock_pixel(SERIAL, "../m1") is None


# ---------------------------------------------------------------------------------------------------------
# UDP loop
# ---------------------------------------------------------------------------------------------------------

class TestServe:
    def test_receives_forwards_and_survives_garbage(self):
        events = []
        hub = live.PoseHub(lambda s, p: events.append(p), lambda ip: SERIAL)
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        fwd.bind(("127.0.0.1", 0))
        fwd.settimeout(2)
        stop = threading.Event()
        th = threading.Thread(target=live.serve, args=(hub, rx, fwd.getsockname(), stop), daemon=True)
        th.start()
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sent = [b"\xff" * 5, packet(0.284, 0, 0), b"", packet(0.284, 0, 0, ver=2), packet(0.3, 0, 0)]
        for d in sent:
            tx.sendto(d, rx.getsockname())
        got = [fwd.recvfrom(4096)[0] for _ in sent]
        assert got == sent                               # raw datagrams, garbage included, in order
        deadline = time.time() + 2
        while not events and time.time() < deadline:
            time.sleep(0.01)
        assert [e["kind"] for e in events] == ["localizing"]
        stop.set()
        th.join(3)
        assert not th.is_alive()
        for s in (rx, fwd, tx):
            s.close()

    def test_tick_runs_while_idle(self):
        events = []
        hub = live.PoseHub(lambda s, p: events.append(p["kind"]), lambda ip: SERIAL)
        hub.handle(packet(0.284, 0, 0), IP, now=time.monotonic() - 10, wall=time.time())
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        stop = threading.Event()
        th = threading.Thread(target=live.serve, args=(hub, rx, None, stop), daemon=True)
        th.start()
        deadline = time.time() + 3
        while "stale" not in events and time.time() < deadline:
            time.sleep(0.05)
        stop.set()
        th.join(3)
        rx.close()
        assert events == ["localizing", "stale"]

    def test_bind_retries_while_the_port_is_busy(self, caplog):
        squatter = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        squatter.bind(("127.0.0.1", 0))
        port = squatter.getsockname()[1]
        stop = threading.Event()
        got = []
        with caplog.at_level("INFO", logger="vacuula_server.livepose"):
            th = threading.Thread(target=lambda: got.append(live.bind_socket("127.0.0.1", port, stop, 0.05)),
                                  daemon=True)
            th.start()
            time.sleep(0.5)                                  # ~10 attempts
            assert not got
            squatter.close()
            th.join(3)
        assert got and got[0] is not None
        got[0].close()
        msgs = [r.getMessage() for r in caplog.records]
        assert len(msgs) == 2 and "retrying" in msgs[0] and "free now" in msgs[1]     # state changes only

    def test_bind_gives_up_when_stopped(self):
        squatter = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        squatter.bind(("127.0.0.1", 0))
        stop = threading.Event()
        res = []
        th = threading.Thread(target=lambda: res.append(live.bind_socket("127.0.0.1", squatter.getsockname()[1],
                                                                          stop, 0.05)), daemon=True)
        th.start()
        time.sleep(0.2)
        stop.set()
        th.join(3)
        squatter.close()
        assert res == [None]

    def test_forward_to_ourselves_is_detected(self):
        assert live.forward_is_self(("127.0.0.1", 7700), "0.0.0.0", 7700)
        assert live.forward_is_self(("127.5.5.5", 7700), "0.0.0.0", 7700)
        assert live.forward_is_self(("0.0.0.0", 7700), "0.0.0.0", 7700)
        assert live.forward_is_self(("192.0.2.5", 7700), "192.0.2.5", 7700)
        assert not live.forward_is_self(("127.0.0.1", 7701), "0.0.0.0", 7700)         # the intended setup
        assert not live.forward_is_self(("192.0.2.9", 7700), "0.0.0.0", 7700)       # another host
        assert not live.forward_is_self(("127.0.0.1", 7700), "192.0.2.5", 7700)     # bound elsewhere: no loop

    def test_start_with_port_zero_binds_nothing_and_starts_no_thread(self, monkeypatch):
        monkeypatch.setattr(live, "POSE_UDP_PORT", 0)

        def boom(*a, **k):
            raise AssertionError("must not be called")
        monkeypatch.setattr(live, "bind_socket", boom)
        monkeypatch.setattr(live.socket, "socket", boom)
        monkeypatch.setattr(live.threading, "Thread", boom)
        before = threading.active_count()
        live.start()
        assert threading.active_count() == before

    def test_start_disables_a_self_forward(self, monkeypatch, caplog):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        monkeypatch.setattr(live, "POSE_UDP_PORT", port)
        monkeypatch.setattr(live, "HOST_IP", "127.0.0.1")
        monkeypatch.setattr(live, "POSE_FORWARD", "127.0.0.1:%d" % port)
        got = {}
        monkeypatch.setattr(live, "bind_socket", lambda *a, **k: sock)
        monkeypatch.setattr(live, "serve", lambda hub, s, forward, stop: got.update(forward=forward))
        with caplog.at_level("INFO", logger="vacuula_server.livepose"):
            live.start()
        sock.close()
        assert got == {"forward": None} and "forwarding is off" in caplog.text

    def test_parse_hostport(self):
        assert live.parse_hostport("127.0.0.1:7701") == ("127.0.0.1", 7701)
        assert live.parse_hostport("") is None
        assert live.parse_hostport(None) is None
        assert live.parse_hostport("7701") is None
        assert live.parse_hostport("127.0.0.1:0") is None
        assert live.parse_hostport("127.0.0.1:x") is None


# ---------------------------------------------------------------------------------------------------------
# Glue with the real server state
# ---------------------------------------------------------------------------------------------------------

class TestGlue:
    @pytest.fixture
    def state(self):
        pytest.importorskip("sqlalchemy")
        from vacuula_server import state as st
        saved = dict(st.robot_connections), list(st.client_connections), live._known_cache
        st.robot_connections.clear()
        st.client_connections.clear()
        live._known_cache = (time.monotonic(), set())          # no database in the tests
        yield st
        st.robot_connections.clear(); st.robot_connections.update(saved[0])
        st.client_connections[:] = saved[1]
        live._known_cache = saved[2]

    def robot(self, serial, ip, gen3=True):
        from vacuula_server.conn import Gen2Gen3Conn, Gen4Conn, RobotConn
        from vacuula_server.generation import Generation
        if gen3:
            return RobotConn(serial, Generation.GEN3, Gen2Gen3Conn("pw", None), ip)
        return RobotConn(serial, Generation.GEN4, Gen4Conn(None))

    def test_resolve_by_connection_ip(self, state):
        state.robot_connections["A-1"] = self.robot("A-1", "10.0.0.1")
        state.robot_connections["B-2"] = self.robot("B-2", "10.0.0.2")
        assert live.resolve_serial("10.0.0.2", "FALLBACK") == "B-2"

    def test_resolve_falls_back_to_setting_then_single_robot(self, state):
        state.robot_connections["A-1"] = self.robot("A-1", "10.0.0.1")
        state.robot_connections["B-2"] = self.robot("B-2", "10.0.0.2")
        assert live.resolve_serial("10.0.0.9", "FALLBACK") == "FALLBACK"
        assert live.resolve_serial("10.0.0.9") is None                 # two robots, no way to tell
        del state.robot_connections["B-2"]
        assert live.resolve_serial("10.0.0.9") == "A-1"                # only one known

    def test_resolve_uses_known_robots_when_nothing_is_connected(self, state):
        live._known_cache = (time.monotonic(), {"DB-1"})
        assert live.resolve_serial("10.0.0.9") == "DB-1"
        live._known_cache = (time.monotonic(), {"DB-1", "DB-2"})
        assert live.resolve_serial("10.0.0.9") is None

    def test_gen4_connections_are_not_matched(self, state):
        state.robot_connections["G4"] = self.robot("G4", None, gen3=False)
        assert live.resolve_serial("10.0.0.9") is None

    def test_publish_reaches_only_that_robots_clients(self, state):
        import queue
        qa, qb = queue.Queue(), queue.Queue()
        state.client_connections.extend([{"client": "t", "serial": "A-1", "queue": qa},
                                         {"client": "t", "serial": "B-2", "queue": qb}])
        live.publish_to_clients("A-1", {"serial": "A-1", "kind": "reset", "ts": 1.5})
        assert qb.empty()
        frame = qa.get_nowait()
        assert frame == b'event: robot_pos\ndata: {"serial": "A-1", "kind": "reset", "ts": 1.5}\n\n'

    def test_publish_keeps_one_pose_per_client_and_never_touches_state_frames(self, state):
        q = state.ClientQueue(maxsize=5)
        state.client_connections.append({"client": "t", "serial": "A-1", "queue": q})
        state.put_drop_oldest(q, b"data: {\"state\": 1}\n\n\n")                      # a rare state frame
        for i in range(50):
            live.publish_to_clients("A-1", {"serial": "A-1", "kind": "pose", "x": i})
        state.put_drop_oldest(q, b"data: {\"state\": 2}\n\n\n")
        frames = [q.get_nowait() for _ in range(q.qsize())]
        assert [f.startswith(b"event: robot_pos") for f in frames] == [False, True, False]
        assert b'"x": 49' in frames[1]                                                 # the latest one won

    def publish_kinds(self, state, q, *kinds):
        """Publish one robot_pos per kind ('pose' kinds get an increasing x), return what the client receives."""
        for i, kind in enumerate(kinds):
            payload = {"serial": "A-1", "ts": float(i), "kind": kind}
            if kind == "pose":
                payload.update(x=float(i), y=0.0, theta=0.0, segment=1, clean=1)
            live.publish_to_clients("A-1", payload)
        out = []
        while not q.empty():
            d = json.loads(q.get_nowait().split(b"data: ", 1)[1])
            out.append((d["kind"], d.get("x")))
        return out

    def test_pending_reset_and_a_new_pose_are_both_delivered_in_order(self, state):
        q = state.ClientQueue(maxsize=10)
        state.client_connections.append({"client": "t", "serial": "A-1", "queue": q})
        assert self.publish_kinds(state, q, "reset", "pose") == [("reset", None), ("pose", 1.0)]

    def test_phase_changes_are_never_replaced_only_poses_collapse(self, state):
        q = state.ClientQueue(maxsize=20)
        state.client_connections.append({"client": "t", "serial": "A-1", "queue": q})
        got = self.publish_kinds(state, q, "pose", "reset", "pose", "pose", "localizing", "pose", "unmapped",
                                 "pose", "stale", "pose", "pose")
        # every phase change, in order; of the poses only the newest one after each survives
        assert [k for k, _ in got if k != "pose"] == ["reset", "localizing", "unmapped", "stale"]
        assert got[-1] == ("pose", 10.0)                       # the very last pose is there
        assert sum(1 for k, _ in got if k == "pose") == 1      # and only one pose waits
        assert got == [("reset", None), ("localizing", None), ("unmapped", None), ("stale", None), ("pose", 10.0)]

    def test_a_pending_phase_change_survives_pose_spam(self, state):
        q = state.ClientQueue(maxsize=5)
        state.client_connections.append({"client": "t", "serial": "A-1", "queue": q})
        got = self.publish_kinds(state, q, "stale", *(["pose"] * 200))
        assert got[0] == ("stale", None) and got[-1] == ("pose", 200.0) and len(got) == 2

    def test_phase_change_on_a_full_queue_is_queued_like_a_state_frame(self, state):
        q = state.ClientQueue(maxsize=3)
        state.client_connections.append({"client": "t", "serial": "A-1", "queue": q})
        for i in range(3):
            q.put_nowait(b"data: {\"state\": %d}\n\n\n" % i)
        live.publish_to_clients("A-1", {"serial": "A-1", "ts": 1.0, "kind": "reset"})   # drops the oldest, as for state
        live.publish_to_clients("A-1", {"serial": "A-1", "ts": 2.0, "kind": "pose", "x": 1.0})   # full: the pose loses
        frames = [q.get_nowait() for _ in range(q.qsize())]
        assert len(frames) == 3 and frames[-1].startswith(b"event: robot_pos") and b'"reset"' in frames[-1]
        assert b'"state": 0' not in b"".join(frames) and b'"pose"' not in b"".join(frames)

    def test_frame_matches_the_existing_event_stream_format(self, state):
        from vacuula_server.handlers import events
        assert live.event_frame("robots", {"a": 1}) == events._frame("robots", {"a": 1})

    def test_frame_is_a_named_sse_event(self):
        frame = live.event_frame(live.EVENT_NAME, {"kind": "pose", "x": 1.0})
        assert frame.endswith(b"\n\n")
        fields = dict(line.split(": ", 1) for line in frame.decode().strip().split("\n"))
        assert fields["event"] == "robot_pos"                 # a named event never reaches EventSource.onmessage
        assert json.loads(fields["data"]) == {"kind": "pose", "x": 1.0}

    def test_full_path_hub_to_client_queue(self, state):
        q = state.ClientQueue()
        state.client_connections.append({"client": "t", "serial": SERIAL, "queue": q})
        state.robot_connections[SERIAL] = self.robot(SERIAL, IP)
        hub = live.PoseHub(live.publish_to_clients, lambda ip: live.resolve_serial(ip))
        hub.handle(packet(0.284, 0, 0), IP, now=1.0, wall=1.0)
        frame = q.get_nowait()
        assert frame.startswith(b"event: robot_pos\ndata: ") and b'"kind": "localizing"' in frame
        assert q.empty()


class TestClientQueue:
    @pytest.fixture
    def q(self):
        pytest.importorskip("sqlalchemy")
        from vacuula_server.state import ClientQueue
        return ClientQueue

    def test_latest_pose_replaces_the_waiting_one(self, q):
        cq = q(maxsize=10)
        assert cq.put_latest(b"event: robot_pos\n1")
        cq.put_nowait(b"data: state\n")
        assert cq.put_latest(b"event: robot_pos\n2")
        assert [cq.get_nowait() for _ in range(cq.qsize())] == [b"data: state\n", b"event: robot_pos\n2"]

    def test_pose_already_taken_is_not_confused_with_a_new_one(self, q):
        cq = q(maxsize=10)
        cq.put_latest(b"event: robot_pos\n1")
        assert cq.get_nowait() == b"event: robot_pos\n1"
        cq.put_nowait(b"data: state\n")
        cq.put_latest(b"event: robot_pos\n2")
        assert cq.qsize() == 2

    def test_full_queue_drops_the_pose_not_the_state_frames(self, q):
        cq = q(maxsize=3)
        for i in range(3):
            cq.put_nowait(b"data: %d\n" % i)
        assert cq.put_latest(b"event: robot_pos\n") is False
        assert [cq.get_nowait() for _ in range(3)] == [b"data: 0\n", b"data: 1\n", b"data: 2\n"]

    def test_pose_evicted_by_drop_oldest_is_harmless(self, q):
        from vacuula_server.state import put_drop_oldest
        cq = q(maxsize=2)
        cq.put_latest(b"event: robot_pos\n1")
        put_drop_oldest(cq, b"data: a\n")
        put_drop_oldest(cq, b"data: b\n")          # the full queue loses its oldest = the pose
        assert cq.put_latest(b"event: robot_pos\n2") is False       # full of state frames: pose dropped
        assert [cq.get_nowait() for _ in range(2)] == [b"data: a\n", b"data: b\n"]


class TestRobotData:
    """gen3.event_handler.robot_data runs in the robot's read loop: live pose must never break it."""

    @pytest.fixture
    def handler(self, monkeypatch):
        pytest.importorskip("sqlalchemy")
        from vacuula_server import state
        from vacuula_server.gen3 import event_handler
        q = state.ClientQueue()
        state.client_connections.append({"client": "t", "serial": "A-1", "queue": q})
        event_handler.robot_buffers.clear()
        monkeypatch.setattr(event_handler, "_last_state_error", float("-inf"))
        yield event_handler, q
        state.client_connections[:] = [c for c in state.client_connections if c["queue"] is not q]

    @staticmethod
    def feed(event_handler, serial, msg):
        body = json.dumps(msg).encode()
        event_handler.robot_data(serial, b"%X\r\n" % (len(body) + 2))
        event_handler.robot_data(serial, body + b"\r\n")

    def test_frame_is_still_delivered_when_note_state_raises(self, handler, monkeypatch, caplog):
        event_handler, q = handler
        monkeypatch.setattr(event_handler, "POSE_UDP_PORT", 7700)

        def boom(serial, msg):
            raise RuntimeError("pose exploded")
        monkeypatch.setattr(event_handler.livepose.hub, "note_state", boom)
        self.feed(event_handler, "A-1", {"eventId": "77", "data": {"state": 2, "action": 11}})
        self.feed(event_handler, "A-1", {"eventId": "77", "data": {"state": 1, "action": 0}})
        assert q.qsize() == 2                                    # both state frames reached the client
        assert caplog.text.count("reading the robot state failed") == 1      # one traceback, not one per frame

    def test_note_state_is_not_called_when_live_pose_is_off(self, handler, monkeypatch):
        event_handler, q = handler
        monkeypatch.setattr(event_handler, "POSE_UDP_PORT", 0)
        calls = []
        monkeypatch.setattr(event_handler.livepose.hub, "note_state", lambda *a: calls.append(a))
        self.feed(event_handler, "A-1", {"eventId": "77", "data": {"state": 2, "action": 11}})
        assert calls == [] and q.qsize() == 1

    def test_note_state_sees_state_frames_when_on(self, handler, monkeypatch):
        event_handler, q = handler
        monkeypatch.setattr(event_handler, "POSE_UDP_PORT", 7700)
        calls = []
        monkeypatch.setattr(event_handler.livepose.hub, "note_state", lambda *a: calls.append(a))
        self.feed(event_handler, "A-1", {"eventId": "77", "data": {"state": 2, "action": 11}})
        assert len(calls) == 1 and calls[0][0] == "A-1"


class TestRoutes:
    @pytest.fixture
    def routes(self):
        pytest.importorskip("sqlalchemy")
        pytest.importorskip("zeroconf")
        from vacuula_server.handlers import route_matcher
        return route_matcher

    def sample(self, pattern):
        return pattern.replace("(.+)", "x").replace("([^/]+)", "x")

    def test_every_route_still_resolves_to_its_own_handler(self, routes):
        """No route shadows another: each route's own sample path finds that very handler (the new /pose included)."""
        for method, pattern, handler in routes.APP_ROUTES:
            found, params = routes.find_route(routes.APP_ROUTES, method, self.sample(pattern))
            assert found is handler, (method, pattern, getattr(found, "__name__", found))

    @pytest.mark.parametrize("path,name", [
        ("/api/robot/S-1/pose", "get_robot_pose"),
        ("/api/robot/S-1/connect", "connect_robot"),
        ("/api/robot/S-1/maps", "get_maps"),
        ("/api/robot/S-1/maps/pose", "get_map"),                 # a map called "pose" is still a map
        ("/api/robot/pose/maps", "get_maps"),                    # and so is a robot called "pose"
        ("/api/robot/S-1/maps/2020-01-01T00:00:00Z", "get_map"),
        ("/api/robot/S-1/runs", "get_runs"),
        ("/api/robot/S-1/run/R", "get_run"),
        ("/api/robots", "get_robots"),
        ("/api/events", "get_events"),
        ("/api/server", "get_server_info"),
    ])
    def test_get_paths(self, routes, path, name):
        found, params = routes.find_route(routes.APP_ROUTES, "GET", path)
        assert found is not None and found.__name__ == name
        if name == "get_robot_pose":
            assert params == ["S-1"]

    def test_pose_only_matches_a_single_serial_segment(self, routes):
        found, _ = routes.find_route(routes.APP_ROUTES, "GET", "/api/robot/a/b/pose")
        assert found is None
        found, _ = routes.find_route(routes.APP_ROUTES, "POST", "/api/robot/S-1/pose")
        assert found is None or found.__name__ != "get_robot_pose"


class TestClientQueueRace:
    def test_put_latest_racing_get(self):
        pytest.importorskip("sqlalchemy")
        from vacuula_server.state import ClientQueue
        q = ClientQueue(maxsize=1000)               # roomy: state frames are never the ones that overflow here
        n_pose, n_state = 5000, 500
        errors, got_pose, got_state = [], [], []
        done = threading.Event()

        def producer():
            try:
                for i in range(n_pose):
                    q.put_latest(b"event: robot_pos\n%d" % i)
                    if i % 10 == 0 and i // 10 < n_state:
                        q.put_nowait(b"data: %d" % (i // 10))
            except Exception as e:          # pragma: no cover
                errors.append(e)
            finally:
                done.set()

        def consumer():
            try:
                while True:
                    try:
                        f = q.get(timeout=0.2)
                    except Exception:
                        if done.is_set() and q.qsize() == 0:
                            return
                        continue
                    (got_pose if f.startswith(b"event") else got_state).append(int(f.split(b"\n")[-1].split(b" ")[-1]))
            except Exception as e:          # pragma: no cover
                errors.append(e)

        threads = [threading.Thread(target=producer), threading.Thread(target=consumer)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(30)
        assert not any(t.is_alive() for t in threads) and not errors
        assert got_state == list(range(n_state))                    # state frames: all, once, in order
        assert got_pose == sorted(set(got_pose)) and got_pose[-1] == n_pose - 1      # latest wins, never reordered
        assert q.qsize() == 0
