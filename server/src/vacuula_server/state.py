from __future__ import annotations
from vacuula_server.conn import RobotConn
from typing import Dict
import threading, queue
from contextlib import suppress

# robot_connections: List[RobotConn]  = []
robot_connections: Dict[str, RobotConn] = {}


# client_connection:
# - client id
# - connection itself
client_connections = []


class PendingRequest:
    __slots__ = ("event", "content", "status")

    def __init__(self):
        self.event = threading.Event()
        self.content = None
        self.status = None

    def fulfill(self, content, status):
        self.content = content
        self.status = status
        self.event.set()

# pending_requests:
# - req_id --> PendingRequest
pending_requests: dict[str, PendingRequest] = {}

class ClientQueue(queue.Queue):
    """Frames waiting for one /connect client.

    Live pose frames (robot_pos kind "pose", up to 5 per second) share this queue with the rare, important robot
    state frames. put_latest keeps at most ONE pose frame waiting (a newer one replaces it) and never evicts anything
    else, so pose traffic can not push a state frame out of a stalled client's queue. Other robot_pos frames
    (localizing, reset, unmapped, stale) are phase changes: they are queued like state frames (put_drop_oldest),
    never replaced by put_latest.
    """

    def __init__(self, maxsize: int = 0):
        super().__init__(maxsize)
        self._pose_frame: bytes | None = None

    def put_latest(self, frame: bytes) -> bool:
        """Queue a pose frame in place of the one still waiting. False if the queue is full (frame dropped)."""
        with self.mutex:
            if self._pose_frame is not None:
                with suppress(ValueError):          # ValueError: the writer already took it
                    self.queue.remove(self._pose_frame)
                    self.unfinished_tasks -= 1
                self._pose_frame = None
            if 0 < self.maxsize <= len(self.queue):
                return False
            self.queue.append(frame)
            self.unfinished_tasks += 1
            self._pose_frame = frame
            self.not_empty.notify()
            return True


def put_drop_oldest(q: queue.Queue, item: bytes) -> None:
    try:
        q.put_nowait(item)
    except queue.Full:
        with suppress(queue.Empty):
            q.get_nowait()
        with suppress(queue.Full):
            q.put_nowait(item)