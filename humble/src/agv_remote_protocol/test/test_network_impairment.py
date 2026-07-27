from agv_remote_protocol.network_impairment import NetworkImpairmentQueue


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_releases_after_configured_delay():
    clock = FakeClock()
    queue = NetworkImpairmentQueue(delay_ms=50.0, clock=clock)

    assert queue.enqueue("command")
    assert queue.pop_ready() == []
    clock.now = 0.049
    assert queue.pop_ready() == []
    clock.now = 0.050
    assert [value for _release_at, value in queue.pop_ready()] == ["command"]
    assert queue.statistics()["released_count"] == 1


def test_preserves_order_with_equal_release_time():
    clock = FakeClock()
    queue = NetworkImpairmentQueue(delay_ms=50.0, clock=clock)
    queue.enqueue("first")
    queue.enqueue("second")

    clock.now = 0.050
    assert [value for _release_at, value in queue.pop_ready()] == [
        "first",
        "second",
    ]


def test_can_drop_every_item_deterministically():
    clock = FakeClock()
    queue = NetworkImpairmentQueue(loss_percent=100.0, clock=clock)

    assert not queue.enqueue("dropped")
    assert queue.pending_count == 0
    assert queue.statistics()["dropped_count"] == 1


def test_negative_jitter_never_creates_negative_delay():
    clock = FakeClock()
    queue = NetworkImpairmentQueue(delay_ms=0.0, jitter_ms=100.0, seed=1, clock=clock)
    queue.enqueue("frame")

    release_at = queue.queue[0][0]
    assert release_at >= clock.now
