#!/usr/bin/env python3
"""Thesis figures from the recorded runs (matplotlib; PDF + PNG).

Writes to --out-dir (default bags/analysis/figures/):

  trajectories.{pdf,png}
      one panel per leg, zone map in grey, every run's map-frame pose
      (/odom composed with map->odom, as in analyze_runs.py), clipped to the
      monitor's active-goal window; one colour per config
  speed_at_zone_entry_single.{pdf,png}, speed_at_zone_entry_grid.{pdf,png}
      goal1 only: /odom speed vs time relative to the first sample inside
      the speed zone; all runs thin, median per config thick; one panel /
      2x2 panels with shared axes

Sized for a 16 cm thesis column, all text >= 9 pt. CONFIG_COLORS is the one
place the per-config colours are defined (colourblind-safe: all pairs
validated for protan/deutan/tritan separation).

Usage:
    python3 plot_runs.py --bags-dir bags --out-dir bags/analysis/figures
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

import analyze_runs as ar  # noqa: E402
from safety_monitor.zone_mask import KEEPOUT, SPEED, ZoneMask  # noqa: E402

CONFIGS = ('base', 'native_keepout', 'native_speed', 'external')
CONFIG_COLORS = {
    'base': '#e34948',            # red
    'native_keepout': '#56b4e9',  # sky blue
    'native_speed': '#eda100',    # yellow
    'external': '#6250d6',        # violet
}
LEGS = ('goal1_zone_entry', 'goal2_zone_exit', 'goal3_keepout')
# Nominal leg start and goal (send_goal.py GOALS; only goal1 starts from the reset pose).
LEG_START = {'goal1_zone_entry': (0.0, 0.0), 'goal2_zone_exit': (0.0, 4.2),
             'goal3_keepout': (-6.0, -3.0)}
LEG_GOAL = {'goal1_zone_entry': (0.0, 4.2), 'goal2_zone_exit': (-6.0, -3.0),
            'goal3_keepout': (-9.0, 0.0)}
ZONE_GREYS = {'free': '#ffffff', 'speed': '#e6e6e6', 'keepout': '#bdbdbd'}
INK = '#2b2b2b'
INK_MUTED = '#6b6b6b'

SPEED_LIMIT = 0.2
SPEED_TOLERANCE = 0.02
ENTRY_WINDOW = (-2.0, 4.0)
ENTRY_GRID = np.arange(ENTRY_WINDOW[0], ENTRY_WINDOW[1] + 1e-9, 0.02)

FIG_WIDTH_IN = 16 / 2.54
CROP_MARGIN_M = 0.5


def setup_style():
    plt.rcParams.update({
        'font.size': 9, 'axes.titlesize': 9, 'axes.labelsize': 9,
        'xtick.labelsize': 9, 'ytick.labelsize': 9, 'legend.fontsize': 9,
        'axes.edgecolor': INK_MUTED, 'axes.labelcolor': INK, 'text.color': INK,
        'xtick.color': INK_MUTED, 'ytick.color': INK_MUTED,
        'axes.spines.top': False, 'axes.spines.right': False,
        'pdf.fonttype': 42, 'ps.fonttype': 42,  # embed TrueType, keeps text editable
        'savefig.dpi': 300, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
    })


def load_runs(bags_dir):
    """config -> leg -> list of runs, each {'samples': full, 'window': (t0, t1)}."""
    runs = {c: {leg: [] for leg in LEGS} for c in CONFIGS}
    for config_id, run_id, run_dir in ar.find_runs(bags_dir):
        if config_id not in runs:
            continue
        goal = ar.load_json(run_dir / 'outcome.json').get('goal')
        leg = ar.LEG_NAMES.get(goal)
        if leg is None:
            continue
        _, map_odom, odom, _ = ar.read_bag(run_dir / 'bag')
        samples = ar.map_frame_samples(odom, map_odom)
        _, _, window, _ = ar.active_window(ar.load_json(run_dir / 'monitor_summary.json'),
                                           odom, samples)
        runs[config_id][leg].append({'run_id': run_id, 'samples': samples, 'window': window})
    return runs


def in_window(run):
    t0, t1 = run['window']
    return [s for s in run['samples'] if t0 <= s[0] <= t1]


def zone_image(zone_mask):
    grid = zone_mask.category_grid
    extent = (zone_mask.origin_x, zone_mask.origin_x + zone_mask.width * zone_mask.resolution,
              zone_mask.origin_y, zone_mask.origin_y + zone_mask.height * zone_mask.resolution)
    # Category ids FREE=0, SPEED=1, KEEPOUT=2 index straight into the colormap.
    cmap = ListedColormap([ZONE_GREYS['free'], ZONE_GREYS['speed'], ZONE_GREYS['keepout']])
    return grid, extent, cmap


def plot_trajectories(runs, zone_mask, out_dir):
    xs, ys = [], []
    for config in CONFIGS:
        for leg in LEGS:
            for run in runs[config][leg]:
                for _, x, y, _ in in_window(run):
                    xs.append(x)
                    ys.append(y)
    for leg in LEGS:
        for x, y in (LEG_START[leg], LEG_GOAL[leg]):
            xs.append(x)
            ys.append(y)
    xlim = (min(xs) - CROP_MARGIN_M, max(xs) + CROP_MARGIN_M)
    ylim = (min(ys) - CROP_MARGIN_M, max(ys) + CROP_MARGIN_M)

    grid, extent, cmap = zone_image(zone_mask)
    panel_w = FIG_WIDTH_IN / 3
    panel_h = panel_w * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    fig, axes = plt.subplots(1, 3, figsize=(FIG_WIDTH_IN, panel_h + 1.0),
                             sharex=True, sharey=True, constrained_layout=True)
    for ax, leg in zip(axes, LEGS):
        ax.imshow(grid, extent=extent, origin='upper', cmap=cmap, vmin=0, vmax=2,
                  interpolation='nearest', zorder=0)
        # Thin outline so the keepout edge stays visible under the light lines.
        ax.contour((grid == KEEPOUT).astype(float), levels=[0.5], extent=extent,
                   origin='upper', colors=INK_MUTED, linewidths=0.6, zorder=1)
        for config in CONFIGS:
            for run in runs[config][leg]:
                pts = in_window(run)
                if pts:
                    ax.plot([p[1] for p in pts], [p[2] for p in pts], color=CONFIG_COLORS[config],
                            lw=0.8, alpha=0.4, solid_capstyle='round', zorder=2)
        sx, sy = LEG_START[leg]
        gx, gy = LEG_GOAL[leg]
        ax.plot(sx, sy, 'o', ms=6, mfc='white', mec=INK, mew=1.2, zorder=3)
        ax.plot(gx, gy, '*', ms=10, mfc=INK, mec='white', mew=0.6, zorder=3)
        ax.set_title(leg)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')

    handles = [Line2D([], [], color=CONFIG_COLORS[c], lw=2, label=c) for c in CONFIGS]
    handles += [
        Patch(facecolor=ZONE_GREYS['speed'], edgecolor=INK_MUTED, lw=0.4, label='speed zone'),
        Patch(facecolor=ZONE_GREYS['keepout'], edgecolor=INK_MUTED, lw=0.6, label='keepout zone'),
        Line2D([], [], ls='', marker='o', ms=6, mfc='white', mec=INK, mew=1.2, label='start'),
        Line2D([], [], ls='', marker='*', ms=10, mfc=INK, mec='white', mew=0.6, label='goal'),
    ]
    fig.legend(handles=handles, loc='outside lower center', ncol=4, frameon=False,
               handlelength=1.6, columnspacing=1.2)
    save(fig, out_dir, 'trajectories')


def entry_curves(runs, zone_mask):
    """config -> list of speed arrays on ENTRY_GRID (NaN outside a run's data)."""
    curves = {}
    for config in CONFIGS:
        curves[config] = []
        for run in runs[config]['goal1_zone_entry']:
            entry = next((t for t, x, y, _ in in_window(run)
                          if zone_mask.category_at(x, y) == SPEED), None)
            if entry is None:
                continue
            t = np.array([s[0] for s in run['samples']]) - entry
            v = np.array([s[3] for s in run['samples']])
            curve = np.interp(ENTRY_GRID, t, v, left=np.nan, right=np.nan)
            curves[config].append(curve)
    return curves


def decorate_entry_axes(ax):
    ax.axvspan(0.0, ENTRY_WINDOW[1], color='#f0f0f0', lw=0, zorder=0)
    ax.axhline(SPEED_LIMIT, color=INK, lw=0.9, zorder=1)
    ax.axhline(SPEED_LIMIT + SPEED_TOLERANCE, color=INK, lw=0.9, ls=(0, (4, 3)), zorder=1)
    ax.set_xlim(*ENTRY_WINDOW)
    ax.set_ylim(0.0, 0.5)


def draw_config(ax, config, curves):
    color = CONFIG_COLORS[config]
    for curve in curves[config]:
        ax.plot(ENTRY_GRID, curve, color=color, lw=0.7, alpha=0.3, zorder=2)
    if curves[config]:
        median = np.nanmedian(np.vstack(curves[config]), axis=0)
        ax.plot(ENTRY_GRID, median, color=color, lw=2.0, zorder=3)


def limit_handles():
    return [
        Line2D([], [], color=INK_MUTED, lw=0.7, alpha=0.5, label='single run'),
        Line2D([], [], color=INK_MUTED, lw=2.0, label='median'),
        Line2D([], [], color=INK, lw=0.9, label=f'limit {SPEED_LIMIT:.2f} m/s'),
        Line2D([], [], color=INK, lw=0.9, ls=(0, (4, 3)),
               label=f'limit + tolerance {SPEED_LIMIT + SPEED_TOLERANCE:.2f} m/s'),
        Patch(facecolor='#f0f0f0', label='inside speed zone'),
    ]


def plot_entry_single(curves, out_dir):
    fig, ax = plt.subplots(figsize=(FIG_WIDTH_IN, 3.0), constrained_layout=True)
    decorate_entry_axes(ax)
    for config in CONFIGS:
        draw_config(ax, config, curves)
    ax.set_xlabel('time relative to speed-zone entry (s)')
    ax.set_ylabel('speed (m/s)')
    handles = [Line2D([], [], color=CONFIG_COLORS[c], lw=2, label=c)
               for c in CONFIGS] + limit_handles()
    fig.legend(handles=handles, loc='outside lower center', ncol=3, frameon=False,
               handlelength=1.8, columnspacing=1.2)
    save(fig, out_dir, 'speed_at_zone_entry_single')


def plot_entry_grid(curves, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(FIG_WIDTH_IN, 4.4), sharex=True, sharey=True,
                             constrained_layout=True)
    for ax, config in zip(axes.flat, CONFIGS):
        decorate_entry_axes(ax)
        draw_config(ax, config, curves)
        ax.set_title(f'{config} (n = {len(curves[config])})')
    for ax in axes[1]:
        ax.set_xlabel('time relative to zone entry (s)')
    for ax in axes[:, 0]:
        ax.set_ylabel('speed (m/s)')
    handles = limit_handles()
    fig.legend(handles=handles, loc='outside lower center', ncol=3, frameon=False,
               handlelength=1.8, columnspacing=1.2)
    save(fig, out_dir, 'speed_at_zone_entry_grid')


def save(fig, out_dir, name):
    for ext in ('pdf', 'png'):
        fig.savefig(out_dir / f'{name}.{ext}')
    plt.close(fig)
    print(f'Wrote {out_dir / name}.pdf/.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bags-dir', default='bags', type=Path)
    parser.add_argument('--out-dir', default='bags/analysis/figures', type=Path)
    maps = ar.PKG_DIR.parent / 'my_robot_sim' / 'maps'
    parser.add_argument('--zone-map-pgm', default=maps / 'zone_map.pgm', type=Path)
    parser.add_argument('--zone-map-yaml', default=maps / 'zone_map.yaml', type=Path)
    args = parser.parse_args()

    setup_style()
    zone_mask = ZoneMask.from_files(str(args.zone_map_pgm), str(args.zone_map_yaml))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    runs = load_runs(args.bags_dir)
    plot_trajectories(runs, zone_mask, args.out_dir)
    curves = entry_curves(runs, zone_mask)
    plot_entry_single(curves, args.out_dir)
    plot_entry_grid(curves, args.out_dir)


if __name__ == '__main__':
    main()
