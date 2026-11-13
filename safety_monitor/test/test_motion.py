"""Stop counting on synthetic speed traces."""
from safety_monitor.motion import count_stops


def trace(*segments, dt=0.1):
    """segments: (duration_s, speed) pieces -> (t, x, y, v) samples."""
    out, t = [], 0.0
    for duration, v in segments:
        for _ in range(int(round(duration / dt))):
            out.append((t, 0.0, 0.0, v))
            t += dt
    return out


def test_pause_between_motion_is_a_stop():
    assert count_stops(trace((2, 0.3), (1.5, 0.0), (2, 0.3))) == 1


def test_short_pause_is_not_a_stop():
    assert count_stops(trace((2, 0.3), (0.8, 0.0), (2, 0.3))) == 0


def test_initial_wait_and_final_rest_are_not_stops():
    assert count_stops(trace((3, 0.0), (2, 0.3), (3, 0.0))) == 0


def test_creeping_below_threshold_counts_as_stopped():
    assert count_stops(trace((1, 0.2), (2, 0.005), (1, 0.2))) == 1
