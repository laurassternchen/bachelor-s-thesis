#!/usr/bin/env python3
"""Offline analysis of recorded runs: safety, mission cost, RQ3 filtering.

Reads every bags/<config_id>/<run_id>/ directory (record_run.sh's bag/,
send_goal.py's outcome.json, safety_monitor_node's monitor_summary.json) and
writes, next to --out:

  runs.csv              one row per run: outcome, mission cost, SPI events,
                        exposure, in-zone speed cross-check, monitor timing
  events.csv            one row per /spi_log event (corrected stamps, distance)
  claim_status.csv      run x safety-case goal: supported/challenged/no_evidence
  claim_summary.csv     status counts per config (all nav goals + per nav goal)
  filtering_stages.csv  data volume per filtering stage, per config
  efficiency.csv        monitor processing time per check cycle, per config
  gsn_<config>.dot      GSN fragment coloured by claim status (+ .svg if
                        Graphviz `dot` is installed)

Everything is clipped to the monitor's active-goal window, using message
header stamps (sim time, the same clock as monitor_summary.json). Exposure is
recomputed here from /odom + /tf + zone_map.pgm, so old and new runs use the
same source. Dependency-light: rosbags, numpy, pyyaml -- no ROS install needed.

Usage:
    python3 analyze_runs.py --bags-dir bags --out bags/analysis/runs.csv
"""
import argparse
import bisect
import copy
import csv
import io
import json
import math
import statistics
import sys
from pathlib import Path

import numpy
import yaml
from rosbags.highlevel import AnyReader

PKG_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_DIR))
from safety_monitor.exposure import (MAX_GAP_S, MAX_PLAUSIBLE_SPEED_MPS,  # noqa: E402
                                      compute_exposure, distance_in_interval)
from safety_monitor.motion import count_stops  # noqa: E402
from safety_monitor.safety_case import STATUSES, SafetyCase, assess_run, summarize  # noqa: E402
from safety_monitor.zone_mask import SPEED, ZoneMask  # noqa: E402

SPI_IDS = ('SPI-1', 'SPI-2', 'SPI-3', 'SPI-4')

# Every circuit is goal1 -> goal2 -> goal3, reset to (0, 0) only before goal1;
# each later leg starts where the previous one ended. Outputs use leg names.
LEG_NAMES = {
    'goal1': 'goal1_zone_entry',  # (0, 0) -> (0, 4.2): ends inside the speed zone
    'goal2': 'goal2_zone_exit',   # (0, 4.2) -> (-6, -3): leaves the speed zone, passes the keepout corner
    'goal3': 'goal3_keepout',     # (-6, -3) -> (-9, 0): around the keepout zone
}

# Filtering stages (RQ3). (a) = every recorded topic except the monitor's own
# outputs; (b) = only what the SPIs read. The action status topic is a hidden
# topic that `ros2 bag record` skips, so it is listed but absent from the bags.
DERIVED_TOPICS = ('/spi_log', '/spi_status')
SPI_INPUT_TOPICS = ('/odom', '/tf', '/tf_static', '/cmd_vel', '/cmd_vel_smoothed',
                    '/navigate_to_pose/_action/status')
# /scan was missing from record_run.sh's topic list for every existing bag.
RAW_NOTE = '/scan not recorded: raw volume is a lower bound'
STAGES = (
    ('a_raw', 'all recorded raw topics'),
    ('b_spi_inputs', 'only the topics the SPIs use'),
    ('c_spi_log', '/spi_log events'),
    ('d_claim_status', 'claim-status rows'),
    ('e_claim_summary', 'claim summary per configuration (config x claim)'),
)

# Runs whose monitor_summary.json carries this convention stamp events at the
# first/last violating sample. Older runs stamped the end at publish time,
# i.e. clear_after_s after the last violating sample.
SAMPLE_STAMP_CONVENTION = 'first_last_violating_sample'
# A legacy event published within this long of the window end was flushed by
# the window closing, not by clear_after_s elapsing -- its end needs no shift.
FINALIZE_SLACK_S = 0.02
ENTRY_BURST_WINDOW_S = 1.0
# A summary whose window lies outside the bag's own /odom span (by more than
# this) belongs to a different goal: older send_goal.py versions sent the goal
# even when the run_id push to the monitor failed, so the window was written
# under the previous run's id.
SUMMARY_MATCH_SLACK_S = 1.0
# Fallback window without a usable summary: span of samples where the robot moves.
MOVING_MPS = 0.01

CLAIM_FIELDS = ('config_id', 'run_id', 'nav_goal', 'pose_reset_ok', 'window_source',
                'goal_id', 'status', 'spi_ids', 'supported_by', 'event_count', 'peak_value',
                'event_duration_s', 'exposure_time_s', 'exposure_distance_m',
                'duration_per_exposure', 'events_per_m')


# -- input ---------------------------------------------------------------------

def load_json(path):
    if not path.exists():
        return {}
    with open(path) as f:
        return json.load(f)


def find_runs(bags_dir):
    for config_dir in sorted(p for p in bags_dir.iterdir() if p.is_dir()):
        for run_dir in sorted(p for p in config_dir.iterdir() if p.is_dir()):
            if (run_dir / 'bag').exists():
                yield config_dir.name, run_dir.name, run_dir


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(bag_dir):
    """One pass: per-topic volume, map->odom transforms, /odom, /spi_log."""
    topic_volume = {}
    map_odom, odom, events = [], [], []
    with AnyReader([bag_dir]) as reader:
        for conn, _, rawdata in reader.messages():
            vol = topic_volume.setdefault(conn.topic, [0, 0])
            vol[0] += 1
            vol[1] += len(rawdata)
            if conn.topic not in ('/tf', '/odom', '/spi_log'):
                continue
            msg = reader.deserialize(rawdata, conn.msgtype)
            if conn.topic == '/tf':
                for tr in msg.transforms:
                    if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                        tl = tr.transform.translation
                        map_odom.append((stamp_s(tr.header.stamp), tl.x, tl.y,
                                         quat_to_yaw(tr.transform.rotation)))
            elif conn.topic == '/odom':
                pos = msg.pose.pose.position
                odom.append((stamp_s(msg.header.stamp), pos.x, pos.y,
                             msg.twist.twist.linear.x))
            else:
                events.append({
                    'spi_id': msg.spi_id, 'zone_id': msg.zone_id,
                    'start_raw': stamp_s(msg.start_stamp), 'end_raw': stamp_s(msg.header.stamp),
                    'peak_value': msg.peak_value, 'threshold': msg.threshold,
                    'x': msg.position.x, 'y': msg.position.y, 'msg_run_id': msg.run_id,
                })
    map_odom.sort()
    odom.sort()
    return topic_volume, map_odom, odom, events


def map_frame_samples(odom, map_odom):
    """(t, x, y, v) in the map frame: each /odom pose composed with the latest
    map->odom transform at or before its stamp (like the monitor's TF lookup)."""
    stamps = [m[0] for m in map_odom]
    out = []
    for t, ox, oy, v in odom:
        i = bisect.bisect_right(stamps, t) - 1
        if i < 0:
            continue
        _, mx, my, yaw = map_odom[i]
        c, s = math.cos(yaw), math.sin(yaw)
        out.append((t, mx + c * ox - s * oy, my + s * ox + c * oy, v))
    return out


# -- per-run analysis -----------------------------------------------------

def correct_events(events, summary, clear_after_s, window):
    """Event start/end = first/last violating sample, clipped to the window."""
    legacy = summary.get('event_stamp_convention') != SAMPLE_STAMP_CONVENTION
    win_start, win_end = window
    out = []
    for ev in events:
        shift = 0.0
        if legacy:
            flushed = (summary.get('window_end_time') is not None
                       and abs(ev['end_raw'] - win_end) <= FINALIZE_SLACK_S)
            shift = 0.0 if flushed else clear_after_s
        start, end = ev['start_raw'] - shift, ev['end_raw'] - shift
        if end < win_start - 1e-3 or start > win_end + FINALIZE_SLACK_S:
            continue
        out.append({**ev, 'start': start, 'end': end, 'duration_s': max(end - start, 0.0),
                    'stamp_shift_s': shift})
    return out


def zone_speed_stats(samples, zone_mask):
    """Real speed while in the speed zone (independent cross-check of SPI-1)."""
    speeds, burst = [], []
    entry_t = None
    for t, x, y, v in samples:
        if zone_mask.category_at(x, y) != SPEED:
            entry_t = None
            continue
        if entry_t is None:
            entry_t = t
        speeds.append(v)
        if t - entry_t <= ENTRY_BURST_WINDOW_S:
            burst.append(v)
    if not speeds:
        return {'speed_zone_sample_count': 0, 'speed_zone_median_mps': None,
                'speed_zone_mean_mps': None, 'speed_zone_p95_mps': None,
                'speed_zone_max_mps': None,
                'speed_zone_entry_peak_mps': None}
    return {'speed_zone_sample_count': len(speeds),
            'speed_zone_median_mps': round(statistics.median(speeds), 4),
            'speed_zone_mean_mps': round(statistics.fmean(speeds), 4),
            'speed_zone_p95_mps': round(float(numpy.percentile(speeds, 95)), 4),
            'speed_zone_max_mps': round(max(speeds), 4),
            'speed_zone_entry_peak_mps': round(max(burst), 4) if burst else None}


def overspeed_exposure(samples, zone_mask, limit):
    """Time and distance driven inside a speed zone faster than `limit`, from
    /odom (sample-and-hold speed, segments classified by midpoint)."""
    time_s = dist_m = 0.0
    for (t0, x0, y0, v0), (t1, x1, y1, _) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0.0 or dt > MAX_GAP_S:
            continue
        ds = math.hypot(x1 - x0, y1 - y0)
        if ds / dt > MAX_PLAUSIBLE_SPEED_MPS or v0 <= limit:
            continue
        if zone_mask.category_at((x0 + x1) / 2.0, (y0 + y1) / 2.0) == SPEED:
            time_s += dt
            dist_m += ds
    return time_s, dist_m


def csv_bytes(rows, fields):
    buf = io.StringIO()
    csv.DictWriter(buf, fieldnames=fields, extrasaction='ignore').writerows(rows)
    return len(buf.getvalue().encode())


def r(value, digits=4):
    return round(value, digits) if isinstance(value, float) else value


def active_window(summary, odom, samples):
    """(summary, summary_mismatch, window, window_source) for one run.

    The returned summary is {} if it belongs to a different goal (see
    SUMMARY_MATCH_SLACK_S). Shared with plot_runs.py.
    """
    summary_mismatch = False
    if summary.get('window_start_time') is not None and odom:
        summary_mismatch = (summary['window_start_time'] < odom[0][0] - SUMMARY_MATCH_SLACK_S
                            or summary['window_end_time'] > odom[-1][0] + SUMMARY_MATCH_SLACK_S)
        if summary_mismatch:
            summary = {}

    moving = [s for s in samples if abs(s[3]) > MOVING_MPS]
    if summary.get('window_start_time') is not None and summary.get('window_end_time') is not None:
        window = (summary['window_start_time'], summary['window_end_time'])
        window_source = 'monitor_summary'
    elif moving:
        window = (moving[0][0], moving[-1][0])
        window_source = 'bag_motion'
    else:
        window, window_source = (0.0, 0.0), 'none'
    return summary, summary_mismatch, window, window_source


def analyze_run(config_id, run_id, run_dir, ctx):
    outcome = load_json(run_dir / 'outcome.json')
    topic_volume, map_odom, odom, raw_events = read_bag(run_dir / 'bag')
    samples = map_frame_samples(odom, map_odom)
    summary, summary_mismatch, window, window_source = active_window(
        load_json(run_dir / 'monitor_summary.json'), odom, samples)
    win_samples = [s for s in samples if window[0] <= s[0] <= window[1]]
    xy = [(t, x, y) for t, x, y, _ in win_samples]

    events = correct_events(raw_events, summary, ctx['clear_after_s'], window)
    for ev in events:
        ev['distance_m'] = distance_in_interval(xy, ev['start'], ev['end'])
        # Penetration no deeper than one map cell: below what the map can resolve.
        ev['within_grid_resolution'] = (ev['spi_id'] == 'SPI-2' and ev['peak_value']
                                        <= ctx['zone_mask'].resolution + 1e-9)
        ev['counts_against_goal'] = (ctx['case'].claim_id(ev['spi_id']) != ''
                                     and ctx['case'].counts_against(ev['spi_id'], ev['peak_value']))
    exposure = compute_exposure(xy, ctx['zone_mask'], ctx['case'].evidence)

    base = {
        'config_id': config_id, 'run_id': run_id,
        'nav_goal': LEG_NAMES.get(outcome.get('goal'), outcome.get('goal')),
        # False: send_goal.py's reset retries were exhausted -- uncontrolled
        # start, exclude when comparing configs. None: reset_pose was false.
        'pose_reset_ok': outcome.get('pose_reset_ok'),
        'window_source': window_source,
    }

    # Claim status (RQ3).
    events_by_spi = {}
    for ev in events:
        events_by_spi.setdefault(ev['spi_id'], []).append(ev)
    claim_rows = [{**base, **{k: r(v) for k, v in row.items()}}
                  for row in assess_run(ctx['case'], events_by_spi, exposure)]

    # Filtering stages (RQ3).
    def volume(topics):
        msgs = sum(topic_volume.get(t, (0, 0))[0] for t in topics)
        size = sum(topic_volume.get(t, (0, 0))[1] for t in topics)
        return msgs, size
    raw_topics = [t for t in topic_volume if t not in DERIVED_TOPICS]
    stages = {
        'a_raw': volume(raw_topics),
        'b_spi_inputs': volume(SPI_INPUT_TOPICS),
        'c_spi_log': volume(['/spi_log']),
        'd_claim_status': (len(claim_rows), csv_bytes(claim_rows, CLAIM_FIELDS)),
    }

    row = {
        **base,
        'accepted': outcome.get('accepted'),
        'pose_reset_distance_m': outcome.get('pose_reset_distance_m'),
        'terminal_status': outcome.get('terminal_status') or summary.get('terminal_status'),
        'error_msg': outcome.get('error_msg'),
        # Goal accepted -> terminal result: the monitor's window, or the motion
        # window where no usable summary exists (see window_source).
        'time_to_goal_s': r(window[1] - window[0], 3) if window_source != 'none' else None,
        'path_length_m': summary.get('path_length_m'),
        # Same for every run (also without a summary): /odom + map->odom over the
        # window, gaps and pose jumps excluded as for exposure.
        'path_length_bag_m': r(distance_in_interval(xy, window[0], window[1]), 3),
        'stop_count': count_stops(win_samples),
        'no_pose_time_s': summary.get('no_pose_time_s'),
        'summary_mismatch': summary_mismatch,
        'window_samples': len(win_samples),
        'legacy_event_stamps': summary.get('event_stamp_convention') != SAMPLE_STAMP_CONVENTION,
        'raw_msg_count': stages['a_raw'][0], 'raw_bytes': stages['a_raw'][1],
        'spi_input_msg_count': stages['b_spi_inputs'][0],
        'spi_input_bytes': stages['b_spi_inputs'][1],
        'spi_log_msg_count': stages['c_spi_log'][0], 'spi_log_bytes': stages['c_spi_log'][1],
        'data_reduction_ratio': (r(stages['a_raw'][0] / stages['c_spi_log'][0], 1)
                                 if stages['c_spi_log'][0] else None),
    }
    for spi_id in SPI_IDS:
        key = spi_id.lower().replace('-', '')
        evs = events_by_spi.get(spi_id, [])
        exp = exposure.get(spi_id, {'time_s': 0.0, 'distance_m': 0.0})
        row[f'{key}_count'] = len(evs)
        row[f'{key}_total_duration_s'] = r(sum(e['duration_s'] for e in evs), 3)
        row[f'{key}_peak_value'] = r(max((e['peak_value'] for e in evs), default=0.0))
        row[f'{key}_event_distance_m'] = r(sum(e['distance_m'] for e in evs), 3)
        row[f'{key}_exposure_time_s'] = r(exp['time_s'], 3)
        row[f'{key}_exposure_distance_m'] = r(exp['distance_m'], 3)
    zone_speed = zone_speed_stats(win_samples, ctx['zone_mask'])
    row.update(zone_speed)
    # Map-based distance to the nearest keepout cell (0 inside keepout), as SPI-2/3 see it.
    on_map = [(x, y) for _, x, y, _ in win_samples if ctx['zone_mask'].category_at(x, y) is not None]
    row['min_keepout_distance_m'] = (r(min(ctx['zone_mask'].clearance_at(x, y) for x, y in on_map), 3)
                                     if on_map else None)
    timing = summary.get('processing_time_ms') or {}
    for check in ('tick', 'spi4_callback'):
        stats = timing.get(check) or {}
        for stat in ('n', 'mean', 'p95', 'max'):
            row[f'{check}_{stat}' + ('' if stat == 'n' else '_ms')] = r(stats.get(stat))

    event_rows = [{
        **base, 'spi_id': e['spi_id'], 'zone_id': e['zone_id'],
        'start_s': r(e['start'], 3), 'end_s': r(e['end'], 3),
        'duration_s': r(e['duration_s'], 3), 'distance_m': r(e['distance_m'], 3),
        'peak_value': r(e['peak_value']), 'threshold': r(e['threshold']),
        'x_m': r(e['x'], 3), 'y_m': r(e['y'], 3), 'msg_run_id': e['msg_run_id'],
        'within_grid_resolution': e['within_grid_resolution'],
        'counts_against_goal': e['counts_against_goal'], 'stamp_shift_s': e['stamp_shift_s'],
    } for e in events]
    detail = {
        **base, 'events': events, 'events_by_spi': events_by_spi, 'exposure': exposure,
        'window': window, 'summary_found': bool(summary),
        'summary_counts': summary.get('spi_event_counts'),
        'bag_spi_log_count': len(raw_events), 'summary_mismatch': summary_mismatch,
        'claims': {c['goal_id']: c for c in claim_rows},
        'zone_speed': zone_speed,
        'overspeed': {tol: overspeed_exposure(win_samples, ctx['zone_mask'],
                                              ctx['case'].threshold('SPI-1') + tol)
                      for tol in ctx['spi1_tolerances']},
    }
    return row, event_rows, claim_rows, stages, timing, detail


# -- aggregation -------------------------------------------------------------

def filtering_rows(stages_by_config, config_stages):
    """stages_by_config: per-run stages a-d; config_stages: config -> stages
    that exist once per config (e)."""
    out = []
    totals = {}
    for config_id, stage_list in sorted(stages_by_config.items()):
        summed = {name: [0, 0] for name, _ in STAGES}
        for stages in stage_list + [config_stages.get(config_id, {})]:
            for name, (msgs, size) in stages.items():
                summed[name][0] += msgs
                summed[name][1] += size
                totals.setdefault(name, [0, 0])
                totals[name][0] += msgs
                totals[name][1] += size
        out += _stage_rows(config_id, len(stage_list), summed)
    if totals:
        out += _stage_rows('ALL', sum(len(v) for v in stages_by_config.values()), totals)
    return out


def _stage_rows(config_id, n_runs, summed):
    rows, prev = [], None
    raw = summed['a_raw']
    for name, desc in STAGES:
        msgs, size = summed[name]
        rows.append({
            'config_id': config_id, 'n_runs': n_runs, 'stage': name, 'description': desc,
            'messages': msgs, 'bytes': size,
            'msg_reduction_vs_prev': r(prev[0] / msgs, 2) if prev and msgs else None,
            'byte_reduction_vs_prev': r(prev[1] / size, 2) if prev and size else None,
            'msg_reduction_vs_raw': r(raw[0] / msgs, 2) if msgs else None,
            'byte_reduction_vs_raw': r(raw[1] / size, 2) if size else None,
            'note': RAW_NOTE if name == 'a_raw' else '',
        })
        prev = (msgs, size)
    return rows


def efficiency_rows(timing_by_config):
    out = []
    for config_id, timings in sorted(timing_by_config.items()):
        for check in ('tick', 'spi4_callback'):
            runs = [t[check] for t in timings if (t.get(check) or {}).get('n')]
            n_total = sum(s['n'] for s in runs)
            out.append({
                'config_id': config_id, 'check': check, 'n_runs_with_timing': len(runs),
                'n_cycles': n_total,
                'mean_ms': r(sum(s['mean'] * s['n'] for s in runs) / n_total, 4) if n_total else None,
                'p95_ms_median_over_runs': r(statistics.median(s['p95'] for s in runs), 4) if runs else None,
                'p95_ms_worst_run': r(max(s['p95'] for s in runs), 4) if runs else None,
                'max_ms': r(max(s['max'] for s in runs), 4) if runs else None,
            })
    return out


def median_or_none(values):
    return r(statistics.median(values)) if values else None


def mean_std(values):
    if not values:
        return None, None
    return (r(statistics.fmean(values), 4),
            r(statistics.stdev(values), 4) if len(values) > 1 else 0.0)


def spi1_overspeed_rows(details, threshold, tolerances):
    """SPI-1 per config and nav goal. Main comparison: time and distance driven
    above limit + tolerance (mean +- std over runs that entered a speed zone);
    the peak speed is secondary (the entry burst is the same in every config)."""
    groups = {}
    for d in details:
        for key in ((d['config_id'], d['nav_goal'] or 'unknown'), (d['config_id'], 'all')):
            groups.setdefault(key, []).append(d)
    out = []
    for (config_id, nav_goal), runs in sorted(groups.items()):
        in_zone = [d for d in runs if d['exposure']['SPI-1']['time_s'] > 0.0]
        peaks = [max(e['peak_value'] for e in d['events_by_spi']['SPI-1'])
                 for d in runs if d['events_by_spi'].get('SPI-1')]
        for tol in tolerances:
            zone_t_mean, zone_t_std = mean_std([d['exposure']['SPI-1']['time_s'] for d in in_zone])
            zone_d_mean, zone_d_std = mean_std([d['exposure']['SPI-1']['distance_m'] for d in in_zone])
            speed_cols = {}
            for stat in ('mean', 'median', 'p95'):
                m, sd = mean_std([d['zone_speed'][f'speed_zone_{stat}_mps'] for d in in_zone
                                  if d['zone_speed'][f'speed_zone_{stat}_mps'] is not None])
                speed_cols[f'speed_in_zone_{stat}_mps_mean'] = m
                speed_cols[f'speed_in_zone_{stat}_mps_std'] = sd
            frac_mean, frac_std = mean_std([d['overspeed'][tol][0] / d['exposure']['SPI-1']['time_s']
                                            for d in in_zone])
            dur_mean, dur_std = mean_std([d['overspeed'][tol][0] for d in in_zone])
            dist_mean, dist_std = mean_std([d['overspeed'][tol][1] for d in in_zone])
            out.append({
                'config_id': config_id, 'nav_goal': nav_goal, 'tolerance_mps': tol,
                'limit_plus_tolerance_mps': r(threshold + tol, 3), 'n_runs': len(runs),
                'n_runs_in_speed_zone': len(in_zone),
                'n_runs_overspeed': sum(1 for d in in_zone if d['overspeed'][tol][0] > 0.0),
                'overspeed_duration_s_mean': dur_mean, 'overspeed_duration_s_std': dur_std,
                'overspeed_distance_m_mean': dist_mean, 'overspeed_distance_m_std': dist_std,
                'time_in_speed_zone_s_mean': zone_t_mean, 'time_in_speed_zone_s_std': zone_t_std,
                'distance_in_speed_zone_m_mean': zone_d_mean,
                'distance_in_speed_zone_m_std': zone_d_std,
                **speed_cols,
                'overspeed_fraction_mean': frac_mean, 'overspeed_fraction_std': frac_std,
                'n_runs_with_spi1_event': len(peaks),
                'peak_speed_max_mps': r(max(peaks), 4) if peaks else None,
                'peak_speed_median_mps': median_or_none(peaks),
                'overspeed_max_mps': r(max(peaks) - threshold, 4) if peaks else None,
            })
    return out


def sensitivity_rows(details, case, spi_id, tolerances):
    """Claim status re-assessed with a different tolerance on one SPI."""
    out = []
    for tol in tolerances:
        varied = copy.deepcopy(case)
        varied.evidence[spi_id]['tolerance'] = tol
        rows = []
        for d in details:
            for claim in assess_run(varied, d['events_by_spi'], d['exposure']):
                rows.append({'config_id': d['config_id'], 'nav_goal': d['nav_goal'] or 'unknown',
                             **claim})
        grouped = summarize([{**c, 'scope': 'all'} for c in rows],
                            ('config_id', 'scope', 'goal_id'))
        grouped += summarize([{**c, 'scope': c['nav_goal']} for c in rows],
                             ('config_id', 'scope', 'goal_id'))
        for g in grouped:
            out.append({'tolerance_spi': spi_id, 'tolerance': tol, **g})
    out.sort(key=lambda g: (g['config_id'], g['scope'] != 'all', g['scope'], g['goal_id'],
                            g['tolerance']))
    return out


def challenged_rows(details, goal_id, spi_id):
    """One row per event that challenges `goal_id` (event-level evidence)."""
    out = []
    for d in details:
        claim = d['claims'].get(goal_id)
        if not claim or claim['status'] != 'challenged':
            continue
        for e in d['events_by_spi'].get(spi_id, []):
            if not d['events'] or not e['counts_against_goal']:
                continue
            out.append({
                'config_id': d['config_id'], 'run_id': d['run_id'], 'nav_goal': d['nav_goal'],
                'pose_reset_ok': d['pose_reset_ok'], 'spi_id': spi_id,
                'start_s': r(e['start'], 3), 'duration_s': r(e['duration_s'], 3),
                'peak_value': r(e['peak_value']),
                'peak_x_m': r(e['x'], 3), 'peak_y_m': r(e['y'], 3),
                'zone_id': e['zone_id'], 'event_distance_m': r(e['distance_m'], 3),
                'within_grid_resolution': e['within_grid_resolution'],
            })
    return out


def source_check_rows(details):
    """Where each run's claim status comes from (own bag /spi_log vs summary)."""
    out = []
    for d in details:
        bag_counts = {s: len(d['events_by_spi'].get(s, [])) for s in SPI_IDS}
        summary_counts = d['summary_counts'] or {}
        out.append({
            'config_id': d['config_id'], 'run_id': d['run_id'], 'nav_goal': d['nav_goal'],
            'window_source': d['window_source'], 'summary_mismatch': d['summary_mismatch'],
            'summary_used': d['summary_found'],
            'bag_spi_log_events': d['bag_spi_log_count'],
            'events_in_window': sum(bag_counts.values()),
            'g3_status': d['claims']['G3']['status'], 'g4_status': d['claims']['G4']['status'],
            'g2_status': d['claims']['G2']['status'],
            'events_by_spi_bag': ' '.join(f'{s}={n}' for s, n in bag_counts.items()),
            'events_by_spi_summary': ' '.join(f'{s}={summary_counts.get(s)}' for s in SPI_IDS)
                                     if summary_counts else '',
            'msg_run_ids': ' '.join(sorted({e['msg_run_id'] for d_e in d['events_by_spi'].values()
                                            for e in d_e})),
        })
    return out


def write_csv(path, rows, fields=None):
    if not rows:
        return
    if fields is None:
        fields = []
        for row in rows:
            fields += [k for k in row if k not in fields]
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    print(f'Wrote {len(rows)} row(s) to {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bags-dir', default='bags', type=Path)
    parser.add_argument('--out', default='runs.csv', type=Path,
                        help='runs.csv path; all other outputs go next to it')
    parser.add_argument('--safety-case', default=PKG_DIR / 'config' / 'safety_case.yaml', type=Path)
    parser.add_argument('--monitor-params', default=PKG_DIR / 'config' / 'safety_monitor.yaml',
                        type=Path, help='read for clear_after_s (legacy stamp correction)')
    maps = PKG_DIR.parent / 'my_robot_sim' / 'maps'
    parser.add_argument('--zone-map-pgm', default=maps / 'zone_map.pgm', type=Path)
    parser.add_argument('--zone-map-yaml', default=maps / 'zone_map.yaml', type=Path)
    parser.add_argument('--spi1-tolerances', nargs='+', type=float, default=[0.0, 0.02],
                        help='SPI-1 tolerances (m/s) for the sensitivity table')
    args = parser.parse_args()

    with open(args.monitor_params) as f:
        params = yaml.safe_load(f)['safety_monitor_node']['ros__parameters']
    ctx = {
        'case': SafetyCase.from_file(args.safety_case),
        'zone_mask': ZoneMask.from_files(str(args.zone_map_pgm), str(args.zone_map_yaml)),
        'clear_after_s': float(params.get('clear_after_s', 0.3)),
        'spi1_tolerances': args.spi1_tolerances,
    }

    run_rows, event_rows, claim_rows = [], [], []
    stages_by_config, timing_by_config, details = {}, {}, []
    for config_id, run_id, run_dir in find_runs(args.bags_dir):
        try:
            row, evs, claims, stages, timing, detail = analyze_run(config_id, run_id, run_dir, ctx)
        except Exception as e:
            # Most commonly: a bag left unfinalized (no metadata.yaml).
            print(f'Skipping {config_id}/{run_id}: {e}')
            continue
        run_rows.append(row)
        event_rows += evs
        claim_rows += claims
        stages_by_config.setdefault(config_id, []).append(stages)
        timing_by_config.setdefault(config_id, []).append(timing)
        details.append(detail)
    if not run_rows:
        print(f'No runs found under {args.bags_dir}')
        return

    out_dir = args.out.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.out, run_rows)
    write_csv(out_dir / 'events.csv', event_rows)
    write_csv(out_dir / 'claim_status.csv', claim_rows, list(CLAIM_FIELDS))
    nav_goal_rows = [{**c, 'nav_goal': c['nav_goal'] or 'unknown'} for c in claim_rows]
    summary = (summarize([{**c, 'nav_goal': 'all'} for c in claim_rows],
                         ('config_id', 'nav_goal', 'goal_id'))
               + summarize(nav_goal_rows, ('config_id', 'nav_goal', 'goal_id')))
    summary.sort(key=lambda s: (s['config_id'], s['nav_goal'] != 'all', s['nav_goal'], s['goal_id']))
    write_csv(out_dir / 'claim_summary.csv', summary)
    summary_fields = ('config_id', 'nav_goal', 'goal_id', 'n_runs') + STATUSES
    config_stages = {}
    for config_id in stages_by_config:
        rows = [s for s in summary if s['config_id'] == config_id and s['nav_goal'] == 'all']
        config_stages[config_id] = {'e_claim_summary': (len(rows), csv_bytes(rows, summary_fields))}
    write_csv(out_dir / 'filtering_stages.csv', filtering_rows(stages_by_config, config_stages))
    eff = efficiency_rows(timing_by_config)
    if any(e['n_cycles'] for e in eff):  # else keep any replay-derived efficiency.csv
        write_csv(out_dir / 'efficiency.csv', eff)
    write_csv(out_dir / 'spi1_overspeed.csv',
              spi1_overspeed_rows(details, ctx['case'].threshold('SPI-1'), args.spi1_tolerances))
    write_csv(out_dir / 'tolerance_sensitivity.csv',
              sensitivity_rows(details, ctx['case'], 'SPI-1', args.spi1_tolerances))
    write_csv(out_dir / 'g3_challenged.csv', challenged_rows(details, 'G3', 'SPI-2'))
    write_csv(out_dir / 'g4_challenged.csv', challenged_rows(details, 'G4', 'SPI-3'))
    write_csv(out_dir / 'source_check.csv', source_check_rows(details))


if __name__ == '__main__':
    main()
