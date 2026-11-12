"""Offline exposure (time/distance) on a synthetic zone mask."""
import numpy as np
import pytest

from safety_monitor.exposure import compute_exposure, distance_in_interval
from safety_monitor.zone_mask import FREE, KEEPOUT, SPEED, ZoneMask

EVIDENCE = {
    'SPI-1': {'exposure': {'kind': 'speed_zone'}},
    'SPI-2': {'exposure': {'kind': 'near_keepout', 'within_m': 1.0}},
    'SPI-3': {'exposure': {'kind': 'whole_run'}},
}


def make_mask():
    # 20 x 4 cells at 0.5 m, origin (0, 0): x in [0, 10), y in [0, 2).
    # Columns 4-7 (x in [2, 4)) = speed, columns 16-19 (x in [8, 10)) = keepout.
    grid = np.full((4, 20), FREE, dtype=np.int8)
    grid[:, 4:8] = SPEED
    grid[:, 16:20] = KEEPOUT
    return ZoneMask(width=20, height=4, resolution=0.5,
                    origin_x=0.0, origin_y=0.0, category_grid=grid)


def straight_line(x_from, x_to, speed=0.5, dt=0.1, y=1.0):
    n = int(round((x_to - x_from) / (speed * dt)))
    return [(i * dt, x_from + i * speed * dt, y) for i in range(n + 1)]


def test_speed_zone_time_and_distance():
    exp = compute_exposure(straight_line(0.0, 6.0), make_mask(), EVIDENCE)
    assert exp['SPI-1']['distance_m'] == pytest.approx(2.0, abs=0.05)
    assert exp['SPI-1']['time_s'] == pytest.approx(4.0, abs=0.1)


def test_whole_run_covers_every_segment():
    exp = compute_exposure(straight_line(0.0, 6.0), make_mask(), EVIDENCE)
    assert exp['SPI-3']['distance_m'] == pytest.approx(6.0, abs=1e-6)
    assert exp['SPI-3']['time_s'] == pytest.approx(12.0, abs=1e-6)


def test_near_keepout_counts_approach_band_only():
    # Clearance is measured cell centre to cell centre: the nearest keepout
    # centre is x = 8.25, so only the cell x in [7.0, 7.5) (centre 7.25,
    # clearance 1.0) is within 1 m on this coarse grid.
    exp = compute_exposure(straight_line(5.0, 7.5), make_mask(), EVIDENCE)
    assert exp['SPI-2']['distance_m'] == pytest.approx(0.5, abs=0.05)
    assert exp['SPI-1']['distance_m'] == 0.0


def test_gaps_longer_than_max_gap_are_ignored():
    samples = [(0.0, 0.0, 1.0), (0.1, 0.05, 1.0), (5.0, 1.0, 1.0)]
    exp = compute_exposure(samples, make_mask(), EVIDENCE)
    assert exp['SPI-3']['time_s'] == pytest.approx(0.1)
    assert exp['SPI-3']['distance_m'] == pytest.approx(0.05)


def test_pose_jumps_are_not_driven_distance():
    # 3 m in 0.1 s = a teleport / re-localisation, not driving.
    samples = [(0.0, 0.5, 1.0), (0.1, 0.55, 1.0), (0.2, 3.55, 1.0), (0.3, 3.6, 1.0)]
    exp = compute_exposure(samples, make_mask(), EVIDENCE)
    assert exp['SPI-3']['distance_m'] == pytest.approx(0.1)
    assert exp['SPI-3']['time_s'] == pytest.approx(0.2)


def test_off_map_samples_are_not_exposure():
    samples = [(0.0, -5.0, -5.0), (0.1, -5.05, -5.0)]
    exp = compute_exposure(samples, make_mask(), EVIDENCE)
    assert exp['SPI-3']['distance_m'] == 0.0


def test_distance_in_interval_prorates_partial_segments():
    samples = straight_line(0.0, 2.0, speed=0.5, dt=0.1)  # 0.05 m per sample
    assert distance_in_interval(samples, 1.0, 2.0) == pytest.approx(0.5)
    assert distance_in_interval(samples, 1.05, 1.15) == pytest.approx(0.05)
    assert distance_in_interval(samples, 2.0, 2.0) == 0.0
