"""SPI-1..4: turn continuous per-tick signals into discrete, debounced events.

Each SPI exposes `update(...) -> dict | None`, returning a completed event
dict only once a violation has stayed clear for `clear_after_s` (not on
every tick the underlying condition changes) -- this is the mechanism that
turns high-volume runtime data into the low-volume, claim-linkable stream
analyze_runs.py and the thesis's safety argument actually consume.

Every SPI reports "magnitude" as a badness score where higher is always
worse, so the shared Debouncer never needs to know which direction is bad
for a given SPI.
"""
from .zone_mask import FREE, KEEPOUT, SPEED  # noqa: F401  (re-exported for callers)


class Debouncer:
    """Generic hysteresis/debounce over a violating/not-violating signal."""

    def __init__(self, clear_after_s=0.3):
        self.clear_after_s = clear_after_s
        self._active = None

    def update(self, t, violating, magnitude, position, zone_id='', zone_type=''):
        if violating:
            if self._active is None:
                self._active = {
                    'start_stamp': t,
                    'peak_value': magnitude,
                    'peak_position': position,
                    'zone_id': zone_id,
                    'zone_type': zone_type,
                    'last_violating_stamp': t,
                }
            else:
                self._active['last_violating_stamp'] = t
                if magnitude > self._active['peak_value']:
                    self._active['peak_value'] = magnitude
                    self._active['peak_position'] = position
            return None
        if self._active is not None and (t - self._active['last_violating_stamp']) >= self.clear_after_s:
            ev = self._active
            self._active = None
            return ev
        return None

    def finalize(self, t):
        if self._active is not None:
            ev = self._active
            self._active = None
            return ev
        return None


class SPI1Overspeed:
    """Robot drives faster than the configured limit inside a speed zone."""
    spi_id = 'SPI-1'

    def __init__(self, claim_id, speed_limit_mps, clear_after_s=0.3):
        self.claim_id = claim_id
        self.threshold = speed_limit_mps
        self.debouncer = Debouncer(clear_after_s)

    def update(self, t, x, y, speed_mps, zone_mask):
        category = zone_mask.category_at(x, y)
        violating = category == SPEED and speed_mps > self.threshold
        magnitude = speed_mps if violating else 0.0
        zone_id = zone_mask.zone_id_at(x, y) if violating else ''
        return self.debouncer.update(t, violating, magnitude, (x, y), zone_id, 'speed')

    def finalize(self, t):
        return self.debouncer.finalize(t)


class SPI2Keepout:
    """Robot's footprint is inside a keepout zone (penetration depth > 0)."""
    spi_id = 'SPI-2'

    def __init__(self, claim_id, clear_after_s=0.3, threshold=0.0):
        self.claim_id = claim_id
        self.threshold = threshold
        self.debouncer = Debouncer(clear_after_s)

    def update(self, t, x, y, zone_mask):
        category = zone_mask.category_at(x, y)
        penetration = zone_mask.penetration_at(x, y) if category == KEEPOUT else 0.0
        violating = category == KEEPOUT and penetration > self.threshold
        magnitude = penetration if violating else 0.0
        zone_id = zone_mask.zone_id_at(x, y) if violating else ''
        return self.debouncer.update(t, violating, magnitude, (x, y), zone_id, 'keepout')

    def finalize(self, t):
        return self.debouncer.finalize(t)


class SPI3Clearance:
    """Robot passes closer to a keepout zone than min_clearance_m (near-miss)."""
    spi_id = 'SPI-3'

    def __init__(self, claim_id, min_clearance_m, clear_after_s=0.3):
        self.claim_id = claim_id
        self.threshold = min_clearance_m
        self.debouncer = Debouncer(clear_after_s)

    def update(self, t, x, y, zone_mask):
        category = zone_mask.category_at(x, y)
        if category == KEEPOUT:
            # Already inside -- SPI-2's domain, not a near-miss.
            return self.debouncer.update(t, False, 0.0, (x, y))
        clearance = zone_mask.clearance_at(x, y)
        violating = clearance < self.threshold
        magnitude = (self.threshold - clearance) if violating else 0.0
        return self.debouncer.update(t, violating, magnitude, (x, y), zone_mask.zone_id_at(x, y), 'free')

    def finalize(self, t):
        return self.debouncer.finalize(t)


class SPI4Intervention:
    """Informational: how much the safety mechanism actually altered the
    commanded velocity (|cmd_vel_smoothed - cmd_vel|). Uniform across all
    four configs since both topics exist identically in each."""
    spi_id = 'SPI-4'

    def __init__(self, claim_id, epsilon_mps=0.02, clear_after_s=0.3):
        self.claim_id = claim_id
        self.threshold = epsilon_mps
        self.debouncer = Debouncer(clear_after_s)

    def update(self, t, x, y, v_smoothed, v_out):
        delta = abs(v_smoothed - v_out)
        violating = delta > self.threshold
        magnitude = delta if violating else 0.0
        return self.debouncer.update(t, violating, magnitude, (x, y))

    def finalize(self, t):
        return self.debouncer.finalize(t)
