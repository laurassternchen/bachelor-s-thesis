"""Offline exposure: how much opportunity each SPI had to fire during a run.

Computed from map-frame samples (t, x, y) that are already clipped to the
active-goal window. Each segment between two consecutive samples is classified
by its midpoint and contributes its duration and length to every exposure kind
it falls under.
"""
import math

from .zone_mask import SPEED

# Gaps longer than this (dropped poses, bag gaps) are not counted as exposure.
MAX_GAP_S = 0.5
# Faster than this is a pose jump (teleport, AMCL re-localisation), not driving.
MAX_PLAUSIBLE_SPEED_MPS = 2.0


def _segments(samples):
    for (t0, x0, y0), (t1, x1, y1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0.0 or dt > MAX_GAP_S:
            continue
        ds = math.hypot(x1 - x0, y1 - y0)
        if ds / dt > MAX_PLAUSIBLE_SPEED_MPS:
            continue
        yield t0, t1, dt, ds, (x0 + x1) / 2.0, (y0 + y1) / 2.0


def _in_exposure(kind, cfg, zone_mask, x, y):
    category = zone_mask.category_at(x, y)
    if category is None:
        return False
    if kind == 'whole_run':
        return True
    if kind == 'speed_zone':
        return category == SPEED
    if kind == 'near_keepout':
        return zone_mask.clearance_at(x, y) <= float(cfg['within_m'])
    raise ValueError(f'unknown exposure kind {kind}')


def compute_exposure(samples, zone_mask, evidence):
    """spi_id -> {'time_s', 'distance_m'} for every evidence entry.

    samples:  time-ordered list of (t, x, y) in the map frame
    evidence: SafetyCase.evidence (spi_id -> entry with an 'exposure' block)
    """
    totals = {spi_id: {'time_s': 0.0, 'distance_m': 0.0} for spi_id in evidence}
    for _, _, dt, ds, mx, my in _segments(samples):
        for spi_id, ev in evidence.items():
            cfg = ev['exposure']
            if _in_exposure(cfg['kind'], cfg, zone_mask, mx, my):
                totals[spi_id]['time_s'] += dt
                totals[spi_id]['distance_m'] += ds
    return totals


def distance_in_interval(samples, t_start, t_end):
    """Distance driven between t_start and t_end, prorating partial segments."""
    if t_end <= t_start:
        return 0.0
    total = 0.0
    for t0, t1, dt, ds, _, _ in _segments(samples):
        overlap = min(t1, t_end) - max(t0, t_start)
        if overlap > 0.0:
            total += ds * overlap / dt
    return total
