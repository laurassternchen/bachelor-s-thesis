"""Offline motion metrics shared by analysis/analyze_runs.py and make_tables.py.

Pure Python. Samples are time-ordered (t, x, y, v) tuples from one active-goal
window, v = /odom forward speed.
"""

# A stop: |v| < STOP_SPEED_MPS for longer than STOP_MIN_S, with motion before
# and after it inside the window -- i.e. not the initial wait for the first
# command and not the final rest at the goal.
STOP_SPEED_MPS = 0.01
STOP_MIN_S = 1.0


def count_stops(samples):
    """Number of stops (see STOP_SPEED_MPS / STOP_MIN_S) in window samples."""
    stops, slow_since, moved = 0, None, False
    for t, _, _, v in samples:
        if abs(v) < STOP_SPEED_MPS:
            if slow_since is None:
                slow_since = t
            continue
        if slow_since is not None and moved and t - slow_since > STOP_MIN_S:
            stops += 1
        slow_since, moved = None, True
    return stops
