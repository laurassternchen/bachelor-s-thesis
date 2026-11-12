#!/usr/bin/env python3
"""Derives keepout_mask_derived and speed_mask_derived from zone_map.pgm.

native_keepout and native_speed each used to load their own, independently
drawn mask (keepout_mask.yaml, reused for both roles) that did not line up
with zone_map.pgm -- the ground truth zone_detector.py and the SafetyMonitor
both read for the "external" config. That meant all four configurations
under test were not actually looking at the same physical zones.

This script reads the single source of truth (maps/zone_map.pgm) and
classifies pixels with the same rule the SafetyMonitor itself uses --
255 = free, 0 = keepout, anything else = speed -- then writes out two
binary (trinary-mode) masks in the existing map_server convention (0 =
occupied/restricted, 255 = free), one per zone type, so KeepoutFilter and
SpeedFilter observe exactly the zones zone_map.pgm defines.

Run once, offline, whenever zone_map.pgm changes:
    python3 scripts/derive_masks.py
"""
import os
import re

MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps')
ZONE_MAP_PGM = os.path.join(MAPS_DIR, 'zone_map.pgm')
ZONE_MAP_YAML = os.path.join(MAPS_DIR, 'zone_map.yaml')
TEMPLATE_YAML = os.path.join(MAPS_DIR, 'keepout_mask.yaml')

OUTPUTS = {
    'keepout': {
        'pgm': os.path.join(MAPS_DIR, 'keepout_mask_derived.pgm'),
        'yaml': os.path.join(MAPS_DIR, 'keepout_mask_derived.yaml'),
        'image_name': 'keepout_mask_derived.pgm',
    },
    'speed': {
        'pgm': os.path.join(MAPS_DIR, 'speed_mask_derived.pgm'),
        'yaml': os.path.join(MAPS_DIR, 'speed_mask_derived.yaml'),
        'image_name': 'speed_mask_derived.pgm',
    },
}


def load_pgm(path):
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
    idx += 1  # single whitespace separator before the pixel data
    if magic == 'P5':
        data = list(raw[idx:idx + width * height])
    else:
        data = list(map(int, raw[idx:].decode('ascii').split()))[:width * height]
    return width, height, maxval, data


def write_pgm_p5(path, width, height, data):
    with open(path, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        f.write(bytes(data))


def load_mask_yaml_template(path):
    """Pulls the scalar fields map_server masks expect, ignoring comments."""
    fields = {}
    with open(path) as f:
        for line in f:
            line = re.sub(r'#.*', '', line).strip()
            if not line or ':' not in line:
                continue
            key, _, value = line.partition(':')
            fields[key.strip()] = value.strip()
    return fields


def write_mask_yaml(path, template_fields, image_name):
    with open(path, 'w') as f:
        f.write(f'image: {image_name}\n')
        f.write(f"resolution: {template_fields['resolution']}\n")
        f.write(f"origin: {template_fields['origin']}\n")
        f.write(f"negate: {template_fields['negate']}\n")
        f.write(f"occupied_thresh: {template_fields['occupied_thresh']}\n")
        f.write(f"free_thresh: {template_fields['free_thresh']}\n")
        f.write('mode: trinary\n')


def classify(pixel):
    """SafetyMonitor's own convention: 255=free, 0=keepout, else=speed."""
    if pixel == 255:
        return 'free'
    if pixel == 0:
        return 'keepout'
    return 'speed'


def main():
    width, height, maxval, data = load_pgm(ZONE_MAP_PGM)
    assert maxval == 255, f'unexpected maxval {maxval}'

    categories = [classify(p) for p in data]
    from collections import Counter
    print('zone_map.pgm category counts:', Counter(categories))

    template_fields = load_mask_yaml_template(TEMPLATE_YAML)

    for zone_type, paths in OUTPUTS.items():
        derived = [0 if cat == zone_type else 255 for cat in categories]
        write_pgm_p5(paths['pgm'], width, height, derived)
        write_mask_yaml(paths['yaml'], template_fields, paths['image_name'])
        print(f"wrote {paths['pgm']} and {paths['yaml']} "
              f"({sum(1 for v in derived if v == 0)} restricted px)")


if __name__ == '__main__':
    main()
