import heapq
import random
import time


class NetworkImpairmentQueue:
    """Delay and optionally drop application-level datagrams or ROS messages."""

    def __init__(
        self,
        delay_ms=0.0,
        jitter_ms=0.0,
        loss_percent=0.0,
        seed=1,
        clock=None,
    ):
        self.delay_ms = max(0.0, float(delay_ms))
        self.jitter_ms = max(0.0, float(jitter_ms))
        self.loss_percent = min(100.0, max(0.0, float(loss_percent)))
        self.clock = clock or time.monotonic
        self.random = random.Random(int(seed))
        self.queue = []
        self.sequence = 0
        self.offered_count = 0
        self.dropped_count = 0
        self.released_count = 0

    def enqueue(self, value):
        self.offered_count += 1
        if self.random.random() * 100.0 < self.loss_percent:
            self.dropped_count += 1
            return False

        jitter = self.random.uniform(-self.jitter_ms, self.jitter_ms)
        effective_delay_ms = max(0.0, self.delay_ms + jitter)
        release_at = self.clock() + effective_delay_ms * 0.001
        heapq.heappush(self.queue, (release_at, self.sequence, value))
        self.sequence += 1
        return True

    def pop_ready(self, now=None):
        ready = []
        deadline = self.clock() if now is None else float(now)
        while self.queue and self.queue[0][0] <= deadline:
            release_at, _sequence, value = heapq.heappop(self.queue)
            ready.append((release_at, value))
            self.released_count += 1
        return ready

    @property
    def pending_count(self):
        return len(self.queue)

    def statistics(self):
        return {
            "delay_ms": self.delay_ms,
            "jitter_ms": self.jitter_ms,
            "loss_percent": self.loss_percent,
            "offered_count": self.offered_count,
            "dropped_count": self.dropped_count,
            "released_count": self.released_count,
            "pending_count": self.pending_count,
        }
