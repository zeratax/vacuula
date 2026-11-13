"""livepose: NEAP v4 decoding, pose filtering and frame transforms for the Neato D7 live position.

Stdlib only. Spec of the wire format ("Version 4"):
https://github.com/zeratax/neato-astro/blob/main/viewer/PACKET.md

Frames
- robot: what the packet carries after localization = the robot's saved-map frame. Metres as decoded with
  5 mm per tic (x_m = raw / 65536 * 0.005); the true tic is ~4.8 mm, which the
  per-map calibration (robot_to_px) absorbs.
  theta = radians, counter-clockwise, (-pi, pi].
- map pixel: Vacuula's Gen3 floor map grid (GET /api/robot/<serial>/maps/<id>), col right, row down.

Stream behaviour the filter handles (verified on recorded streams):
- Packets start at the first clean after boot. Before the robot localizes it sits still at exactly
  (284 mm, 0, 0) in a dock-relative frame, then jumps (two steps, ~0.4 s apart) into the map frame.
- While docked after a clean (and after the robot is lifted) the pose is exactly (0, 0, 0) and packets keep coming
  at ~18/s. "docked" itself must come from the robot state, not from the pose.
- A relocalization during a run shows up as a jump; the trail starts a new segment there.
- flags bit 3 (torn) = the 64-word window changed during every copy attempt; such packets are dropped.
"""
import math
import struct
import time

NEAP_MAGIC = 0x4E454150
BASE_FMT = "<IHHIIiiiBBBB"
BASE_LEN = struct.calcsize(BASE_FMT)      # 32
TIC_M = 0.005
FIX = 65536.0
BAM = 2 * math.pi / 4294967296.0


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def decode(data):
    """datagram -> dict or None. Only version >= 4 carries the smooth pose in BASE x/y/theta."""
    if len(data) < BASE_LEN:
        return None
    magic, ver, flags, seq, t_ms, x, y, th, _mi, _mc, _loc, _r = struct.unpack_from(BASE_FMT, data, 0)
    if magic != NEAP_MAGIC or ver < 4:
        return None
    th &= 0xFFFFFFFF
    return {
        "seq": seq, "t_ms": t_ms, "flags": flags,
        "x": x / FIX * TIC_M, "y": y / FIX * TIC_M, "theta": _wrap(th * BAM),
        "zero": x == 0 and y == 0 and th == 0,
        "torn": bool(flags & 8),
    }


class Affine2D:
    """p' = A p + b with A = [[a, b], [d, e]], b = (c, f).  Headings rotate by the angle of A's first column
    (and flip sign if the transform is a reflection)."""

    def __init__(self, a, b, c, d, e, f):
        self.m = (float(a), float(b), float(c), float(d), float(e), float(f))
        self.det = a * e - b * d
        self.rot = math.atan2(d, a)

    @classmethod
    def from_rows(cls, row_x, row_y):
        """from [a, b, c] and [d, e, f] (x' = a x + b y + c, y' = d x + e y + f)"""
        return cls(*row_x, *row_y)

    def apply(self, x, y):
        a, b, c, d, e, f = self.m
        return a * x + b * y + c, d * x + e * y + f

    def heading(self, theta):
        """direction angle in the target frame, measured CCW from its +x axis (in that frame's own handedness)"""
        a, b, c, d, e, f = self.m
        ux, uy = math.cos(theta), math.sin(theta)
        return math.atan2(d * ux + e * uy, a * ux + b * uy)


class PoseFilter:
    """Turns decoded packets into display state.

    feed(pkt, now) -> one of
      None                      nothing worth publishing (dropped packet / no change)
      {"kind": "pose", ...}     localized pose in the robot map frame: x, y, theta, segment, clean
      {"kind": "reset"}         pose reset to exactly (0,0,0): docked OR lifted (take "docked" from the robot state)
      {"kind": "localizing"}    stream active, pose still in the pre-localization (dock-relative) frame
      {"kind": "unmapped"}      the current clean does not use the saved map (set_map_clean(False))
    Non-pose kinds are reported once per phase. `clean` counts cleans (a non-zero pose after a reset starts a new
    one): consumers reset their trail when it changes. self.trail is always the current clean's trail.

    Map-frame gate: call set_map_clean(True/False/None) from the robot state (Gen3 state: cleaning.category == 4 =
    map clean; 2 house, 3 spot; exploration builds a new map). The normal clean start (static in the dock frame,
    then a localization jump) works with None. The "joined mid-run" fallback (pose moved smoothly > 0.30 m with no
    jump seen) is used only when the receiver started during a map clean: map_clean is True and no reset has been
    seen yet. False never localizes: such poses are not in the saved-map frame.
    """

    JUMP_M, V_MAX = 0.15, 0.5                          # displacement > JUMP_M + V_MAX*dt (m) cannot be driving
    JUMP_RAD, W_MAX = math.radians(20), math.radians(150)   # same for heading (measured turn rate ~72 deg/s)
    GAP_S = 1.0              # no packet for this long while localized: break the trail (no line across the gap)
    SETTLE_S = 1.0           # localization arrives as two jumps ~0.4 s apart
    SMOOTH_LOCALIZED_M = 0.30
    TRAIL_STEP_M = 0.03
    TRAIL_MAX = 6000

    def __init__(self):
        self.map_clean = None
        self.last_rx = None
        self.clean = 0
        self._seen_reset = False
        self.reset_state()

    def set_map_clean(self, flag):
        self.map_clean = flag

    def reset_state(self):
        self.localized = False
        self.at_reset = False
        self.pose = None             # (x, y, theta) in the map frame, last localized
        self._prev = None            # last accepted raw pose (any frame)
        self._anchor = None          # first pose of the current pre-localization phase
        self._jump_at = None
        self._said = None
        self.segment = 0
        self.trail = []              # current clean: list of segments, each a list of (x, y)

    def _once(self, kind):
        if self._said == kind:
            return None
        self._said = kind
        return {"kind": kind}

    def _new_segment(self):
        self.segment += 1
        self.trail.append([])

    def _add_trail(self, x, y):
        if not self.trail:
            self.trail.append([])
        seg = self.trail[-1]
        if not seg or math.hypot(x - seg[-1][0], y - seg[-1][1]) >= self.TRAIL_STEP_M:
            seg.append((x, y))
            total = sum(len(s) for s in self.trail)
            while total > self.TRAIL_MAX and self.trail:
                drop = min(len(self.trail[0]), total - self.TRAIL_MAX)
                del self.trail[0][:drop]
                if not self.trail[0]:
                    del self.trail[0]
                total -= drop

    def feed(self, pkt, now=None):
        now = time.monotonic() if now is None else now
        if pkt is None or pkt["torn"]:
            return None
        self.last_rx = now
        if pkt["zero"]:
            if not self.at_reset:
                self.at_reset = True
                self._seen_reset = True
                self.localized = False
                self._prev = self._anchor = self._jump_at = None
            return self._once("reset")
        if self.at_reset or self.clean == 0:          # first non-zero pose after a reset (or ever): a new clean
            self.at_reset = False
            self.clean += 1
            self.trail, self.segment = [], 0
        x, y, th = pkt["x"], pkt["y"], pkt["theta"]
        prev, self._prev = self._prev, (x, y, th, now)
        if self.map_clean is False:
            self.localized = False
            self._anchor = self._jump_at = None
            return self._once("unmapped")
        if not self.localized and self._anchor is None:
            self._anchor = (x, y)
        if prev is None:
            return None if self.localized else self._once("localizing")
        dt = max(0.0, now - prev[3])
        jump = math.hypot(x - prev[0], y - prev[1]) > self.JUMP_M + self.V_MAX * dt \
            or abs(_wrap(th - prev[2])) > self.JUMP_RAD + self.W_MAX * dt
        if not self.localized:
            if jump:
                self._jump_at = now
                return self._once("localizing")
            if self._jump_at is not None:
                if now - self._jump_at < self.SETTLE_S:
                    return self._once("localizing")
            elif not (self.map_clean is True and not self._seen_reset
                      and math.hypot(x - self._anchor[0], y - self._anchor[1]) > self.SMOOTH_LOCALIZED_M):
                return self._once("localizing")
            self.localized = True
            self._new_segment()
        elif jump or dt > self.GAP_S:
            self._new_segment()             # relocalization or a gap: keep the new pose, start a new trail segment
        self._said = "pose"
        self.pose = (x, y, th)
        self._add_trail(x, y)
        return {"kind": "pose", "x": x, "y": y, "theta": th, "segment": self.segment, "clean": self.clean}

    def stale(self, now=None, after_s=3.0):
        now = time.monotonic() if now is None else now
        return self.last_rx is None or now - self.last_rx > after_s


class Throttle:
    """Rate-limit published poses: at most one per `min_interval_s`, and only if moved / turned / heartbeat due."""

    def __init__(self, min_interval_s=1.0, min_move_m=0.02, min_turn_rad=math.radians(3), heartbeat_s=30.0):
        self.min_interval_s, self.min_move_m, self.min_turn_rad, self.heartbeat_s = \
            min_interval_s, min_move_m, min_turn_rad, heartbeat_s
        self._last = None
        self._last_t = None

    def due(self, x, y, theta, now=None, force=False):
        now = time.monotonic() if now is None else now
        if self._last is None or force:
            self._last, self._last_t = (x, y, theta), now
            return True
        if now - self._last_t < self.min_interval_s:
            return False
        lx, ly, lt = self._last
        if math.hypot(x - lx, y - ly) >= self.min_move_m or abs(_wrap(theta - lt)) >= self.min_turn_rad \
                or now - self._last_t >= self.heartbeat_s:
            self._last, self._last_t = (x, y, theta), now
            return True
        return False

