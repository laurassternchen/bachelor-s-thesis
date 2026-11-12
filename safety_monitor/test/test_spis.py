"""SPI-1 hysteresis/debounce and SPI-2 penetration depth on synthetic data."""
import numpy as np

from safety_monitor.spis import Debouncer, SPI1Overspeed, SPI2Keepout
from safety_monitor.zone_mask import FREE, KEEPOUT, SPEED, ZoneMask


def make_zone_mask():
    # 10x10 cells, 0.5 m resolution, origin (0, 0): row 0 (top) = keepout,
    # row 9 (bottom) = speed, everything else free.
    grid = np.full((10, 10), FREE, dtype=np.int8)
    grid[0, :] = KEEPOUT
    grid[9, :] = SPEED
    return ZoneMask(width=10, height=10, resolution=0.5,
                    origin_x=0.0, origin_y=0.0, category_grid=grid)


# -- Debouncer --------------------------------------------------------------

def test_debouncer_no_event_while_violating():
    d = Debouncer(clear_after_s=0.3)
    assert d.update(0.0, True, 1.0, (0, 0)) is None
    assert d.update(0.1, True, 2.0, (0, 0)) is None


def test_debouncer_emits_after_clear_delay():
    d = Debouncer(clear_after_s=0.3)
    d.update(0.0, True, 1.0, (0, 0))
    d.update(0.1, True, 2.0, (1, 1))   # peak here
    assert d.update(0.2, False, 0.0, (2, 2)) is None       # not clear long enough yet
    ev = d.update(0.1 + 0.3, False, 0.0, (2, 2))
    assert ev is not None
    assert ev['start_stamp'] == 0.0
    assert ev['peak_value'] == 2.0
    assert ev['peak_position'] == (1, 1)


def test_debouncer_resumes_within_hysteresis_window():
    d = Debouncer(clear_after_s=0.3)
    d.update(0.0, True, 1.0, (0, 0))
    d.update(0.2, False, 0.0, (0, 0))   # brief dip, well under clear_after_s
    d.update(0.25, True, 5.0, (9, 9))   # violating again -> same episode, new peak
    ev = d.update(0.25 + 0.3, False, 0.0, (0, 0))
    assert ev is not None
    assert ev['start_stamp'] == 0.0       # episode never actually closed
    assert ev['peak_value'] == 5.0


def test_debouncer_finalize_flushes_active_event():
    d = Debouncer(clear_after_s=0.3)
    d.update(0.0, True, 3.0, (0, 0))
    assert d.update(0.1, True, 3.0, (0, 0)) is None
    ev = d.finalize(0.1)
    assert ev is not None
    assert ev['peak_value'] == 3.0


# -- SPI-1 overspeed ----------------------------------------------------------

def test_spi1_fires_in_speed_zone_above_limit():
    zm = make_zone_mask()
    spi = SPI1Overspeed('C-SPI-1', speed_limit_mps=0.2, clear_after_s=0.3)
    assert spi.update(0.0, 1.0, 0.25, 0.5, zm) is None       # violating, no event yet
    assert spi.update(0.1, 1.0, 0.25, 0.8, zm) is None       # still violating, peak=0.8
    assert spi.update(0.2, 1.0, 0.25, 0.0, zm) is None       # dropped below limit, not clear yet
    ev = spi.update(0.2 + 0.3, 1.0, 0.25, 0.0, zm)
    assert ev is not None
    assert ev['peak_value'] == 0.8
    assert ev['zone_type'] == 'speed'


def test_spi1_never_fires_in_free_zone():
    zm = make_zone_mask()
    spi = SPI1Overspeed('C-SPI-1', speed_limit_mps=0.2, clear_after_s=0.3)
    for t in (0.0, 0.1, 0.2, 0.5, 0.8):
        assert spi.update(t, 1.0, 2.5, 5.0, zm) is None
    assert spi.finalize(1.0) is None


def test_spi1_never_fires_below_limit():
    zm = make_zone_mask()
    spi = SPI1Overspeed('C-SPI-1', speed_limit_mps=0.2, clear_after_s=0.3)
    for t in (0.0, 0.1, 0.2):
        assert spi.update(t, 1.0, 0.25, 0.1, zm) is None
    assert spi.finalize(0.5) is None


# -- SPI-2 keepout penetration depth -----------------------------------------

def test_spi2_peak_value_is_penetration_depth():
    zm = make_zone_mask()
    spi = SPI2Keepout('C-SPI-2', clear_after_s=0.3)
    # Entire top row is keepout, so penetration depth there is the distance
    # to the nearest non-keepout cell (row 1), i.e. one cell = 0.5 m.
    assert spi.update(0.0, 1.0, 4.75, zm) is None
    ev = spi.finalize(0.0)
    assert ev is not None
    assert ev['peak_value'] == 0.5
    assert ev['zone_type'] == 'keepout'


def test_spi2_never_fires_outside_keepout():
    zm = make_zone_mask()
    spi = SPI2Keepout('C-SPI-2', clear_after_s=0.3)
    for t in (0.0, 0.1, 0.2):
        assert spi.update(t, 1.0, 2.5, zm) is None
    assert spi.finalize(0.5) is None
