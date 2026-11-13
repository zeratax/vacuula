"""Live robot position.

A patched Neato D7 robot sends its SLAM pose as UDP "NEAP v4" datagrams (~18 per second, only while
cleaning). This module listens for them (POSE_UDP_PORT, 0 = off), filters them with livepose_core and hands
the result to the web app:

- a named SSE event `robot_pos` on the robot's `/api/robot/<serial>/connect` stream (<= 5 poses/s, state
  changes immediately). Named events never reach EventSource.onmessage, so existing clients are unaffected.
- `GET /api/robot/<serial>/pose`: latest status, pose, trail and the map calibration.

Poses are in the robot's own map frame (metres). `<data dir>/livepose.json` says how that frame maps onto the
pixels of a persistent map (see Calibration). Optionally every raw datagram is forwarded to POSE_FORWARD
("host:port") so another recorder can keep listening.

Status / event kinds: pose, localizing (stream active, pose not in the map frame yet), reset (pose is exactly
(0,0,0): docked or lifted), unmapped (this clean does not use the saved map), stale (no packet for 3 s); /pose
also says none. Whether a clean uses the saved map comes from the Gen3 robot state (note_state).

Everything here must keep working on garbage input: the UDP thread never dies on a bad packet.
"""
from __future__ import annotations

import json
import logging
import math
import socket
import threading
import time
from collections.abc import Callable
from pathlib import Path

from vacuula_server import livepose_core as lp
from vacuula_server.constants import DATA_DIR, MAPS_DIR
from vacuula_server.env import HOST_IP, POSE_FORWARD, POSE_SERIAL, POSE_UDP_PORT

logger = logging.getLogger(__name__)

EVENT_NAME = "robot_pos"
PUBLISH_MIN_INTERVAL_S = 0.2     # <= 5 pose events per second per robot
PUBLISH_HEARTBEAT_S = 2.0        # a robot standing still still sends a pose this often (the app greys out after 5 s)
STALE_AFTER_S = 3.0              # no packet for this long -> `stale`
RESTART_AFTER_S = 60.0           # silence this long = the robot restarted its stream, start from scratch
TICK_S = 0.5                     # how often the receive loop checks for staleness
BIND_RETRY_S = 30.0              # POSE_UDP_PORT busy (another listener): try again this often
DOCK_CHECK_PX = 8.0              # on reset, the last pose may be this far from the map's dock before we complain
CALIBRATION_FILE = DATA_DIR / "livepose.json"

Publish = Callable[[str, dict], None]
ResolveSerial = Callable[[str], "str | None"]
DockPixel = Callable[[str, str], "tuple[float, float] | None"]


def event_frame(event: str, data) -> bytes:
    """One SSE frame, same shape as handlers.events._frame (the /connect writer sends queue items verbatim)."""
    return f"event: {event}\ndata: {json.dumps(data)}\n\n".encode()


# ---------------------------------------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------------------------------------

# No built-in calibration: a map is only drawn once livepose.json has an entry for its id (see Calibration).
DEFAULT_CALIBRATION: dict = {}


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def parse_calibration(doc) -> dict:
    """`{"maps": {"<map id>": {"robot_to_px": [[a,b,c],[d,e,f]]}}}` -> the valid maps only."""
    maps: dict = {}
    entries = doc.get("maps") if isinstance(doc, dict) else None
    if not isinstance(entries, dict):
        raise ValueError('expected {"maps": {...}}')
    for map_id, entry in entries.items():
        rows = entry.get("robot_to_px") if isinstance(entry, dict) else None
        if isinstance(rows, list) and len(rows) == 2 and all(
            isinstance(r, list) and len(r) == 3 and all(_is_number(v) for v in r) for r in rows
        ):
            maps[str(map_id)] = {"robot_to_px": [[float(v) for v in r] for r in rows]}
        else:
            logger.warning("livepose.json: map %s needs robot_to_px = [[a,b,c],[d,e,f]], ignored", map_id)
    return maps


class Calibration:
    """Robot map frame -> persistent map pixels, from `<data dir>/livepose.json`.

    px_col = a*x + b*y + c, px_row = d*x + e*y + f   (x, y in metres)

    Read on first use and again whenever the file's mtime changes (checked at most once per `check_every_s`).
    Without a file no map has a transform and no robot is drawn; a file that
    cannot be parsed keeps whatever was valid before.
    """

    def __init__(self, path: Path = CALIBRATION_FILE, check_every_s: float = 1.0):
        self.path = Path(path)
        self.check_every_s = check_every_s
        self._maps: dict = DEFAULT_CALIBRATION
        self._mtime: int | None = None      # mtime_ns of the file the current maps came from
        self._checked = -math.inf
        self._lock = threading.Lock()

    def get(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        with self._lock:
            if now - self._checked >= self.check_every_s:
                self._checked = now
                self._refresh()
            return {"maps": self._maps}

    def _refresh(self) -> None:
        try:
            mtime = self.path.stat().st_mtime_ns
        except OSError:
            if self._mtime is not None:
                logger.info("livepose.json is gone, no map has a calibration")
            self._maps, self._mtime = DEFAULT_CALIBRATION, None
            return
        if mtime == self._mtime:
            return
        self._mtime = mtime          # remember even if broken: one warning per change, not one per request
        try:
            self._maps = parse_calibration(json.loads(self.path.read_text("utf-8")))
            logger.info("livepose.json loaded: %d map(s)", len(self._maps))
        except (OSError, ValueError) as e:
            logger.warning("livepose.json not usable (%s), keeping the previous calibration", e)


def read_dock_pixel(serial: str, map_id: str) -> tuple[float, float] | None:
    """Dock marker of a persistent map in cell coordinates: floormap.json's dock pixel_x / pixel_y (raw, as drawn)."""
    if "/" in map_id or ".." in map_id:
        return None
    try:
        doc = json.loads((MAPS_DIR / serial / map_id / "floormap.json").read_text("utf-8"))
        dock = doc["dock"]
        return float(dock["pixel_x"]), float(dock["pixel_y"])
    except (OSError, ValueError, KeyError, TypeError):
        return None


# ---------------------------------------------------------------------------------------------------------
# Robot state (Gen3): is the current clean using the saved map?
# ---------------------------------------------------------------------------------------------------------

STATE_BUSY, STATE_PAUSED = 2, 3
# exploring a new map, acquiring ids / creating + uploading a map, suspended exploration: not a clean on the saved map
NOT_MAP_ACTIONS = (12, 13, 14, 15)
CATEGORY_MANUAL, CATEGORY_HOUSE, CATEGORY_SPOT, CATEGORY_MAP = 1, 2, 3, 4


def state_body(msg) -> dict | None:
    """The robot state inside a Gen3 message: an eventId 77 frame (state under `data`) or a getRobotState answer."""
    if not isinstance(msg, dict):
        return None
    for body in (msg.get("data"), msg):
        if isinstance(body, dict) and "state" in body and "action" in body:
            return body
    return None


def map_clean_from_state(body: dict) -> bool | None:
    """True = map clean (poses are in the saved-map frame), False = something else (house/spot/manual cleaning,
    exploring a new map: other frame), None = idle or unknown."""
    if body.get("state") not in (STATE_BUSY, STATE_PAUSED):
        return None
    if body.get("action") in NOT_MAP_ACTIONS:
        return False
    cleaning = body.get("cleaning")
    category = cleaning.get("category") if isinstance(cleaning, dict) else None
    if category == CATEGORY_MAP:
        return True
    if category in (CATEGORY_MANUAL, CATEGORY_HOUSE, CATEGORY_SPOT):
        return False
    return None


# ---------------------------------------------------------------------------------------------------------
# Hub: datagrams in, events + snapshots out
# ---------------------------------------------------------------------------------------------------------

def _new_throttle() -> lp.Throttle:
    return lp.Throttle(min_interval_s=PUBLISH_MIN_INTERVAL_S, heartbeat_s=PUBLISH_HEARTBEAT_S)


class _Track:
    """Live pose state of one robot."""

    def __init__(self, map_clean: bool | None = None):
        self.filter = lp.PoseFilter()
        self.filter.set_map_clean(map_clean)
        self.throttle = _new_throttle()
        self.status = "none"            # none | pose | localizing | reset | unmapped | stale
        self.ts: float | None = None    # unix time of the last status change / accepted pose
        self.last_pose: dict | None = None   # last localized pose as published (x, y, theta, segment, clean)


class PoseHub:
    def __init__(self, publish: Publish, resolve_serial: ResolveSerial, calibration: Calibration | None = None,
                 dock_pixel: DockPixel | None = None):
        self._publish = publish
        self._resolve = resolve_serial
        self.calibration = calibration or Calibration()
        self._dock_pixel = dock_pixel or read_dock_pixel
        self._tracks: dict[str, _Track] = {}
        self._map_clean: dict[str, bool | None] = {}     # from the robot state, may arrive before any packet
        self._lock = threading.Lock()
        self._warned: dict[str, float] = {}

    def _warn_every(self, key: str, seconds: float, msg: str, *args) -> None:
        now = time.monotonic()
        if now - self._warned.get(key, -math.inf) >= seconds:
            self._warned[key] = now
            logger.warning(msg, *args)

    def set_map_clean(self, serial: str, flag: bool | None) -> None:
        """Does the robot's current clean use the saved map? True / False / None = unknown (see PoseFilter)."""
        with self._lock:
            self._map_clean[serial] = flag
            if serial in self._tracks:
                self._tracks[serial].filter.set_map_clean(flag)

    def note_state(self, serial: str, msg) -> None:
        """Feed a Gen3 message (eventId 77 state frame or getRobotState answer); anything else is ignored."""
        body = state_body(msg)
        if body is not None:
            self.set_map_clean(serial, map_clean_from_state(body))

    def handle(self, data: bytes, ip: str, now: float | None = None, wall: float | None = None) -> None:
        """Process one datagram from `ip`. Bad packets are dropped silently (decode returns None)."""
        pkt = lp.decode(data)
        if pkt is None:
            return
        serial = self._resolve(ip)
        if serial is None:
            self._warn_every("serial:" + ip, 60, "pose packets from %s cannot be matched to a robot "
                                                 "(set POSE_SERIAL)", ip)
            return
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        out: dict | None = None
        reset_pose = None
        with self._lock:
            t = self._tracks.get(serial)
            if t is None:
                t = self._tracks[serial] = _Track(self._map_clean.get(serial))
                logger.info("Live pose: first packet from %s, robot %s", ip, serial)
            f = t.filter
            if f.last_rx is not None and now - f.last_rx > RESTART_AFTER_S:
                # the robot went quiet for a long time: its stream restarts in the pre-localization frame
                f.reset_state()
                f.clean += 1
                t.last_pose = None
                t.throttle = _new_throttle()
            ev = f.feed(pkt, now)
            if ev is None:
                return
            kind = ev["kind"]
            if kind == "pose":
                after_gap = t.status != "pose"      # first pose after localizing / stale / reset: show it now
                t.status, t.ts = "pose", wall
                t.last_pose = {"x": round(ev["x"], 4), "y": round(ev["y"], 4), "theta": round(ev["theta"], 4),
                               "segment": ev["segment"], "clean": ev["clean"]}
                if t.throttle.due(ev["x"], ev["y"], ev["theta"], now, force=after_gap):
                    out = {"kind": "pose", **t.last_pose}
            else:
                t.status, t.ts = kind, wall
                out = {"kind": kind}
                if kind == "reset":
                    reset_pose = f.pose
        if out is not None:
            self._emit(serial, out, wall)
        if reset_pose is not None:
            self._check_dock(serial, reset_pose)

    def _check_dock(self, serial: str, pose: tuple[float, float, float]) -> None:
        """On a reset the robot sits on (or near) its dock: the last pose should land on the map's dock marker."""
        try:
            for map_id, entry in self.calibration.get()["maps"].items():
                dock = self._dock_pixel(serial, map_id)
                if dock is None:
                    continue
                (a, b, c), (d, e, f) = entry["robot_to_px"]
                col, row = a * pose[0] + b * pose[1] + c, d * pose[0] + e * pose[1] + f
                dist = math.hypot(col - dock[0], row - dock[1])
                if dist > DOCK_CHECK_PX:
                    logger.warning("Live pose: reset at px (%.1f, %.1f) but map %s has its dock at (%.1f, %.1f): "
                                   "%.1f px apart (> %.0f). Calibration drift, or the robot was lifted.",
                                   col, row, map_id, dock[0], dock[1], dist, DOCK_CHECK_PX)
                else:
                    logger.info("Live pose: reset %.1f px from the dock of map %s (calibration ok)", dist, map_id)
        except Exception:
            logger.exception("Live pose: dock check failed")

    def tick(self, now: float | None = None, wall: float | None = None) -> None:
        """Mark robots whose stream went quiet as `stale` (once). A reset (docked) robot stays that way."""
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        gone = []
        with self._lock:
            for serial, t in self._tracks.items():
                if t.status in ("pose", "localizing", "unmapped") and t.filter.stale(now, STALE_AFTER_S):
                    t.status, t.ts = "stale", wall
                    gone.append(serial)
        for serial in gone:
            self._emit(serial, {"kind": "stale"}, wall)

    def _emit(self, serial: str, payload: dict, wall: float) -> None:
        try:
            self._publish(serial, {"serial": serial, "ts": round(wall, 3), **payload})
        except Exception:
            logger.exception("Live pose: publishing to the web clients failed")

    def snapshot(self, serial: str, now: float | None = None, wall: float | None = None) -> dict:
        """Body of GET /api/robot/<serial>/pose."""
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        status, ts, pose, trail = "none", None, None, []
        with self._lock:
            t = self._tracks.get(serial)
            if t is not None:
                f = t.filter
                status, ts = t.status, t.ts
                if status in ("pose", "localizing", "unmapped") and f.stale(now, STALE_AFTER_S):
                    status = "stale"        # tick() has not run yet
                pose = None if t.last_pose is None else dict(t.last_pose)
                trail = [[[round(x, 3), round(y, 3)] for x, y in seg] for seg in f.trail if seg]
        return {
            "serial": serial,
            "status": status,
            "ts": None if ts is None else round(ts, 3),
            "now": round(wall, 3),          # server clock, so a client can age `ts` without trusting its own
            "pose": pose,
            "trail": trail,
            "calibration": self.calibration.get(now),
        }


# ---------------------------------------------------------------------------------------------------------
# Glue to the rest of the server (lazy imports: state/generation pull in the database layer)
# ---------------------------------------------------------------------------------------------------------

_known_cache: tuple[float, set[str]] = (-math.inf, set())


def _known_gen3_serials() -> set[str]:
    """Gen3 robots the server knows about: connected ones plus the ones in the database (cached 30 s)."""
    global _known_cache
    from vacuula_server.generation import Generation
    from vacuula_server.state import robot_connections

    serials = {s for s, c in list(robot_connections.items()) if c.gen == Generation.GEN3}
    stamp, cached = _known_cache
    if time.monotonic() - stamp > 30:
        try:
            from sqlalchemy import select
            from vacuula_server.db import get_session
            from vacuula_server.models.robot import Robot
            with get_session() as session:
                cached = set(session.execute(select(Robot.serial).where(Robot.type == "neato.gen3")).scalars())
        except Exception as e:
            logger.warning("Live pose: could not list robots from the database: %s", e)
        _known_cache = (time.monotonic(), cached)
    return serials | cached


def resolve_serial(ip: str, fallback_serial: str | None = None) -> str | None:
    """Datagram source IP -> robot serial.

    1. the connected Gen3 robot whose TCP connection comes from that IP,
    2. POSE_SERIAL,
    3. the only Gen3 robot the server knows.
    """
    from vacuula_server.generation import Generation
    from vacuula_server.state import robot_connections

    for serial, conn in list(robot_connections.items()):
        if conn.gen == Generation.GEN3 and conn.addr == ip:
            return serial
    if fallback_serial:
        return fallback_serial
    known = _known_gen3_serials()
    return next(iter(known)) if len(known) == 1 else None


def publish_to_clients(serial: str, payload: dict) -> None:
    """Queue a robot_pos frame for every web client connected to that robot's /connect stream."""
    from vacuula_server import state

    clients = [c for c in list(state.client_connections) if c["serial"] == serial]
    if not clients:
        return
    frame = event_frame(EVENT_NAME, payload)
    is_pose = payload.get("kind") == "pose"
    for client in clients:
        # Only pose frames are "latest wins" (at most one waits per client, it never pushes a state frame out).
        # localizing / reset / unmapped / stale are phase changes a client must not miss: queued like a state frame.
        put = getattr(client["queue"], "put_latest", None)
        if is_pose and put is not None:
            put(frame)
        else:
            state.put_drop_oldest(client["queue"], frame)


hub = PoseHub(publish_to_clients, lambda ip: resolve_serial(ip, POSE_SERIAL))


def snapshot(serial: str) -> dict:
    return hub.snapshot(serial)


# ---------------------------------------------------------------------------------------------------------
# UDP receive loop
# ---------------------------------------------------------------------------------------------------------

def parse_hostport(value: str | None) -> tuple[str, int] | None:
    """"host:port" -> (ip, port); None (and a warning) when unusable."""
    if not value:
        return None
    host, _, port = value.strip().rpartition(":")
    try:
        if not host or not 1 <= int(port) <= 65535:
            raise ValueError
        return socket.gethostbyname(host), int(port)
    except (ValueError, OSError):
        logger.warning("POSE_FORWARD must be host:port (IPv4), got %r - forwarding is off", value)
        return None


def _local_addresses() -> set[str]:
    addrs = {"127.0.0.1", "0.0.0.0"}
    try:
        import ifaddr
        for adapter in ifaddr.get_adapters():
            addrs.update(ip.ip for ip in adapter.ips if ip.is_IPv4 and isinstance(ip.ip, str))
    except Exception:
        pass            # loopback is the case that matters
    return addrs


def forward_is_self(forward: tuple[str, int], bind_host: str, bind_port: int) -> bool:
    """Would forwarding to `forward` feed our own socket (an endless loop pinning the CPU)?"""
    ip, port = forward
    if port != bind_port:
        return False
    if bind_host in ("", "0.0.0.0"):
        return ip.startswith("127.") or ip in _local_addresses()
    return ip == bind_host


def serve(pose_hub: PoseHub, sock: socket.socket, forward: tuple[str, int] | None = None,
          stop: threading.Event | None = None) -> None:
    """Receive datagrams on `sock` until `stop` is set. Never raises on bad input."""
    stop = stop or threading.Event()
    sock.settimeout(TICK_S)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if forward else None
    errors: dict[str, float] = {}

    def log_error(key: str, msg: str, *args) -> None:
        if time.monotonic() - errors.get(key, -math.inf) >= 60:
            errors[key] = time.monotonic()
            logger.warning(msg, *args, exc_info=True)

    next_tick = 0.0
    while not stop.is_set():
        data = addr = None
        try:
            data, addr = sock.recvfrom(4096)
        except TimeoutError:
            pass
        except OSError as e:
            if stop.is_set():
                break
            log_error("recv", "Live pose: receive failed: %s", e)
            stop.wait(0.5)
            continue
        if data is not None and addr is not None:
            if out is not None and forward is not None:
                try:
                    out.sendto(data, forward)       # best effort, a missing recorder must not matter
                except OSError:
                    pass
            try:
                pose_hub.handle(data, addr[0])
            except Exception:
                log_error("handle", "Live pose: packet from %s dropped", addr[0])
        if time.monotonic() >= next_tick:
            next_tick = time.monotonic() + TICK_S
            try:
                pose_hub.tick()
            except Exception:
                log_error("tick", "Live pose: staleness check failed")
    if out is not None:
        out.close()


def bind_socket(host: str, port: int, stop: threading.Event, retry_s: float = BIND_RETRY_S) -> socket.socket | None:
    """Bind the pose port, retrying while it is busy (e.g. another receiver still owns it). Logs on changes only."""
    waiting = False
    while not stop.is_set():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((host, port))     # no SO_REUSEADDR: never share the port with another listener
        except OSError as e:
            sock.close()
            if not waiting:
                waiting = True
                logger.warning("Live pose: cannot listen on udp://%s:%d (%s), retrying every %d s",
                               host, port, e, retry_s)
            stop.wait(retry_s)
            continue
        if waiting:
            logger.info("Live pose: udp://%s:%d is free now", host, port)
        return sock
    return None


def start() -> None:
    """Thread target: bind POSE_UDP_PORT (retrying while busy) and serve."""
    if POSE_UDP_PORT <= 0:
        return
    stop = threading.Event()
    sock = bind_socket(HOST_IP, POSE_UDP_PORT, stop)
    if sock is None:
        return
    forward = parse_hostport(POSE_FORWARD)
    if forward is not None and forward_is_self(forward, HOST_IP, POSE_UDP_PORT):
        logger.error("POSE_FORWARD %s is this server's own pose port, forwarding is off", POSE_FORWARD)
        forward = None
    logger.info("Live pose listening on udp://%s:%d (forward: %s, fallback robot: %s)",
                HOST_IP, POSE_UDP_PORT, POSE_FORWARD if forward else "off", POSE_SERIAL or "none")
    serve(hub, sock, forward, stop)
