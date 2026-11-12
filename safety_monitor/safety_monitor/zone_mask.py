"""Ground-truth zone classification, read directly from a map_server mask.

The monitor must stay identical across all four configurations under test,
including `base`, which has no zone-related node running at all -- so it
cannot subscribe to /zone_truth (only published by zone_map_server in the
"external" config's own launch file). Instead it reads the same
zone_map.pgm/.yaml file directly from disk and classifies pixels itself,
using the exact convention zone_map.yaml documents: 255 = free, 0 =
keepout, anything else = speed.
"""
import heapq
from collections import deque

import numpy as np
import yaml

FREE = 0
SPEED = 1
KEEPOUT = 2

CATEGORY_NAMES = {FREE: 'free', SPEED: 'speed', KEEPOUT: 'keepout'}


def _load_pgm(path):
    """Loads a PGM in either P2 (ASCII) or P5 (binary) form."""
    with open(path, 'rb') as f:
        raw = f.read()
    magic = raw[:2].decode('ascii')
    if magic not in ('P2', 'P5'):
        raise ValueError(f'unsupported PGM magic {magic!r} in {path}')
    idx = 2
    values = []
    while len(values) < 3:
        while raw[idx] in b' \t\r\n':
            idx += 1
        if raw[idx:idx + 1] == b'#':
            while raw[idx] not in b'\r\n':
                idx += 1
            continue
        start = idx
        while raw[idx] not in b' \t\r\n':
            idx += 1
        values.append(int(raw[start:idx]))
    width, height, maxval = values
    idx += 1
    if magic == 'P5':
        data = list(raw[idx:idx + width * height])
    else:
        data = list(map(int, raw[idx:].decode('ascii').split()))[:width * height]
    return width, height, maxval, data


def _classify(pixel):
    if pixel == 255:
        return FREE
    if pixel == 0:
        return KEEPOUT
    return SPEED


def _label_blobs(category_grid, target):
    """4-connected flood fill; returns an int grid, -1 where not `target`."""
    h, w = category_grid.shape
    labels = np.full((h, w), -1, dtype=np.int32)
    next_label = 0
    for sr in range(h):
        for sc in range(w):
            if category_grid[sr, sc] != target or labels[sr, sc] != -1:
                continue
            q = deque([(sr, sc)])
            labels[sr, sc] = next_label
            while q:
                r, c = q.popleft()
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nr, nc = r + dr, c + dc
                    if (0 <= nr < h and 0 <= nc < w
                            and category_grid[nr, nc] == target and labels[nr, nc] == -1):
                        labels[nr, nc] = next_label
                        q.append((nr, nc))
            next_label += 1
    return labels


def _multi_source_distance(category_grid, resolution, source_mask):
    """Multi-source Dijkstra distance (metres) from every cell to the
    nearest cell where `source_mask` is True -- an 8-connected
    approximation of the Euclidean distance transform."""
    h, w = category_grid.shape
    dist = np.full((h, w), np.inf, dtype=np.float64)
    heap = []
    for r, c in zip(*np.where(source_mask)):
        dist[r, c] = 0.0
        heap.append((0.0, int(r), int(c)))
    heapq.heapify(heap)
    steps = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 2 ** 0.5), (-1, 1, 2 ** 0.5), (1, -1, 2 ** 0.5), (1, 1, 2 ** 0.5))
    while heap:
        d, r, c = heapq.heappop(heap)
        if d > dist[r, c]:
            continue
        for dr, dc, step in steps:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w:
                nd = d + step
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    heapq.heappush(heap, (nd, nr, nc))
    return dist * resolution


class ZoneMask:
    """Static, file-loaded zone classification for one map.

    Row/col convention (matches raw PGM raster order, row 0 = top = max y):
        world_x = origin_x + (col + 0.5) * resolution
        world_y = origin_y + (height - row - 0.5) * resolution
    """

    def __init__(self, width, height, resolution, origin_x, origin_y, category_grid):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.category_grid = category_grid
        self.keepout_labels = _label_blobs(category_grid, KEEPOUT)
        self.speed_labels = _label_blobs(category_grid, SPEED)
        self.clearance_grid = _multi_source_distance(
            category_grid, resolution, category_grid == KEEPOUT)
        self.penetration_grid = _multi_source_distance(
            category_grid, resolution, category_grid != KEEPOUT)

    @classmethod
    def from_files(cls, pgm_path, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        width, height, maxval, data = _load_pgm(pgm_path)
        assert maxval == 255, f'unexpected maxval {maxval} in {pgm_path}'
        pixels = np.array(data, dtype=np.int16).reshape(height, width)
        category_grid = np.vectorize(_classify)(pixels).astype(np.int8)
        origin_x, origin_y = meta['origin'][0], meta['origin'][1]
        return cls(width, height, meta['resolution'], origin_x, origin_y, category_grid)

    def world_to_cell(self, x, y):
        col = int(np.floor((x - self.origin_x) / self.resolution))
        row = self.height - 1 - int(np.floor((y - self.origin_y) / self.resolution))
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def category_at(self, x, y):
        """Returns one of FREE/SPEED/KEEPOUT, or None if (x, y) is off-map."""
        cell = self.world_to_cell(x, y)
        if cell is None:
            return None
        return int(self.category_grid[cell])

    def zone_id_at(self, x, y):
        cell = self.world_to_cell(x, y)
        if cell is None:
            return ''
        r, c = cell
        cat = int(self.category_grid[r, c])
        if cat == KEEPOUT:
            return f'keepout_{int(self.keepout_labels[r, c])}'
        if cat == SPEED:
            return f'speed_{int(self.speed_labels[r, c])}'
        return ''

    def clearance_at(self, x, y):
        """Metres to the nearest keepout cell; 0.0 if already inside one."""
        cell = self.world_to_cell(x, y)
        if cell is None:
            return 0.0
        return float(self.clearance_grid[cell])

    def penetration_at(self, x, y):
        """Metres from the keepout boundary to (x, y); 0.0 if outside keepout."""
        cell = self.world_to_cell(x, y)
        if cell is None:
            return 0.0
        return float(self.penetration_grid[cell])
