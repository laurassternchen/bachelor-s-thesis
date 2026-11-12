"""World<->pixel lookup and category classification on a synthetic .pgm.

Matches ZoneMask's own documented convention (row 0 = top = max y) and
correction 1's from-disk classification rule: 255=free, 0=keepout, else=speed.
"""
import os

import pytest

from safety_monitor.zone_mask import FREE, KEEPOUT, SPEED, ZoneMask


def _write_synthetic_map(tmp_path):
    # 10x10 cells, 0.5 m resolution, origin (0, 0) -> 5x5 m map.
    # Row 0 (top, y near 5) is keepout; row 9 (bottom, y near 0) is speed;
    # everything else is free.
    width, height, resolution = 10, 10, 0.5
    pixels = [[255] * width for _ in range(height)]
    for c in range(width):
        pixels[0][c] = 0      # top row -> keepout
        pixels[9][c] = 128    # bottom row -> speed (any non-0/255 value)

    pgm_path = tmp_path / 'synthetic.pgm'
    with open(pgm_path, 'w') as f:
        f.write(f'P2\n{width} {height}\n255\n')
        for row in pixels:
            f.write(' '.join(str(v) for v in row) + '\n')

    yaml_path = tmp_path / 'synthetic.yaml'
    with open(yaml_path, 'w') as f:
        f.write(
            'image: synthetic.pgm\n'
            'mode: scale\n'
            f'resolution: {resolution}\n'
            'origin: [0.0, 0.0, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n'
        )
    return str(pgm_path), str(yaml_path)


@pytest.fixture
def zone_mask(tmp_path):
    pgm_path, yaml_path = _write_synthetic_map(tmp_path)
    return ZoneMask.from_files(pgm_path, yaml_path)


def test_dimensions(zone_mask):
    assert (zone_mask.width, zone_mask.height, zone_mask.resolution) == (10, 10, 0.5)


def test_top_row_is_keepout(zone_mask):
    # Top row (row 0) covers y in [4.5, 5.0); 4.75 is its cell centre.
    assert zone_mask.category_at(1.0, 4.75) == KEEPOUT


def test_bottom_row_is_speed(zone_mask):
    # Bottom row (row 9) covers y in [0.0, 0.5); 0.25 is its cell centre.
    assert zone_mask.category_at(1.0, 0.25) == SPEED


def test_middle_is_free(zone_mask):
    assert zone_mask.category_at(1.0, 2.5) == FREE


def test_off_map_is_none(zone_mask):
    assert zone_mask.category_at(-5.0, -5.0) is None


def test_world_to_cell_row_flip(zone_mask):
    # Highest y maps to row 0 (top of the raster), lowest y to the last row.
    assert zone_mask.world_to_cell(1.0, 4.9) == (0, 2)
    assert zone_mask.world_to_cell(1.0, 0.1) == (9, 2)


def test_penetration_zero_outside_keepout(zone_mask):
    assert zone_mask.penetration_at(1.0, 2.5) == 0.0


def test_clearance_zero_inside_keepout(zone_mask):
    assert zone_mask.clearance_at(1.0, 4.75) == 0.0


def test_clearance_positive_in_free_space(zone_mask):
    assert zone_mask.clearance_at(1.0, 2.5) > 0.0
