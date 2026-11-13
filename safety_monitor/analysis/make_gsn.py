#!/usr/bin/env python3
"""GSN figures from config/safety_case.yaml (Graphviz `dot`; PDF + PNG + DOT).

Writes to --out-dir (default bags/analysis/figures/):

  gsn_structure.{dot,pdf,png}  structure only: goals, strategy, solutions,
                               context (thresholds/tolerances), no numbers
  gsn_<config>.{dot,pdf,png}   same layout, goals coloured by claim status and
                               the supported / challenged / no-evidence tally
                               over all legs of that config (appendix)

Claim counts come from tolerance_sensitivity.csv at --spi1-tolerance, the
same source as the claim-status table (make_tables.py).

Usage:
    python3 make_gsn.py --analysis-dir bags/analysis
"""
import argparse
import csv
import subprocess
import sys
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_DIR))
from safety_monitor.safety_case import FILL_LEGEND, STATUSES, SafetyCase, to_dot  # noqa: E402

# Text is 10 pt at natural size; at most this wide, so a figure scaled to a
# 15 cm text width never shrinks it below 9 pt.
MAX_WIDTH_PT = 15 / 2.54 * 72 / 0.9


def render(dot_path):
    for fmt, extra in (('pdf', []), ('png', ['-Gdpi=300'])):
        subprocess.run(['dot', f'-T{fmt}', *extra, str(dot_path), '-o',
                        str(dot_path.with_suffix(f'.{fmt}'))], check=True)


def natural_width_pt(dot_path):
    out = subprocess.run(['dot', '-Tplain', str(dot_path)], check=True,
                         capture_output=True, text=True).stdout
    return float(out.splitlines()[0].split()[2]) * 72  # "graph scale width height", inches


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--analysis-dir', default='bags/analysis', type=Path)
    parser.add_argument('--out-dir', type=Path, help='default: <analysis-dir>/figures')
    parser.add_argument('--safety-case', default=PKG_DIR / 'config' / 'safety_case.yaml',
                        type=Path)
    parser.add_argument('--spi1-tolerance', default=0.02, type=float)
    args = parser.parse_args()
    out_dir = args.out_dir or args.analysis_dir / 'figures'
    out_dir.mkdir(parents=True, exist_ok=True)

    case = SafetyCase.from_file(args.safety_case)
    tolerances = {'SPI-1': args.spi1_tolerance}
    figures = {'gsn_structure': to_dot(case, tolerances=tolerances)}

    with open(args.analysis_dir / 'tolerance_sensitivity.csv') as f:
        rows = [r for r in csv.DictReader(f) if r['scope'] == 'all'
                and float(r['tolerance']) == args.spi1_tolerance]
    for config_id in sorted({r['config_id'] for r in rows}):
        counts = {r['goal_id']: {s: int(r[s]) for s in STATUSES}
                  for r in rows if r['config_id'] == config_id}
        n_legs = max(int(r['n_runs']) for r in rows if r['config_id'] == config_id)
        title = (f'{case.id}: {config_id}, {n_legs} legs, SPI-1 tolerance '
                 f'{args.spi1_tolerance:g} m/s (S / C / N = supported / challenged / no '
                 f'evidence; {FILL_LEGEND})')
        figures[f'gsn_{config_id}'] = to_dot(case, counts, title=title, tolerances=tolerances)

    for name, dot in figures.items():
        dot_path = out_dir / f'{name}.dot'
        dot_path.write_text(dot)
        render(dot_path)
        width = natural_width_pt(dot_path)
        flag = '' if width <= MAX_WIDTH_PT else f'  (wider than {MAX_WIDTH_PT:.0f} pt!)'
        print(f'Wrote {out_dir / name}.dot/.pdf/.png, natural width {width:.0f} pt{flag}')


if __name__ == '__main__':
    main()
