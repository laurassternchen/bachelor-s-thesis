#!/usr/bin/env python3
"""Replay recorded bags through the live SafetyMonitor (RQ3 efficiency + check).

For one bag per (config, nav goal) this starts safety_monitor.launch.py with
use_sim_time:=true and its output remapped to /spi_log_replay, plays the bag's
/odom, /tf, /tf_static, /cmd_vel and /cmd_vel_smoothed with `ros2 bag play
--clock`, and collects what the monitor publishes. Two results:

  efficiency.csv         monitor processing time per check cycle (the node's own
                         processing_time_ms), per config -- no new sim runs needed
  replay_comparison.csv  per bag: events recomputed by the monitor during replay
                         vs events recorded live (same corrections as
                         analyze_runs.py), event by event

Window times and replayed event stamps are converted between sim time and the
replay clock (see ClockMap). The bags do not hold /navigate_to_pose/_action/status (hidden topic), so this
script opens and closes the monitor's goal window itself by publishing a
synthetic GoalStatusArray at the recorded window start/end (monitor_summary.json,
sim time). Everything runs on a private ROS_DOMAIN_ID so a running sim is not
disturbed.

Needs a sourced ROS 2 + workspace and the `rosbags` Python package.

Usage:
    python3 replay_bags.py --bags-dir bags --out-dir bags/analysis
"""
import argparse
import bisect
import csv
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(PKG_DIR))

REPLAY_DOMAIN_ID = '87'
SPI_LOG_REPLAY = '/spi_log_replay'
STATUS_TOPIC = '/navigate_to_pose/_action/status'
PLAYED_TOPICS = ['/odom', '/tf', '/tf_static', '/cmd_vel', '/cmd_vel_smoothed']
SPI_IDS = ('SPI-1', 'SPI-2', 'SPI-3', 'SPI-4')
STATUS_CODES = {'succeeded': 4, 'canceled': 5, 'aborted': 6}
MONITOR_STARTUP_S = 5.0
SETTLE_S = 2.0
# Legacy recorded stamps are only recovered to within a tick or two.
STAMP_TOLERANCE_S = 0.1


class ClockMap:
    """Sim time (message header stamps) <-> replay clock (bag record time).

    `ros2 bag play --clock` publishes /clock from the time each message was
    *recorded* (wall time), whereas every header stamp and monitor_summary.json
    window is sim time. /odom ties the two together message by message.
    """

    def __init__(self, bag_dir):
        from rosbags.highlevel import AnyReader
        sim, rec = [], []
        with AnyReader([bag_dir]) as reader:
            for conn, ts, raw in reader.messages(
                    connections=[c for c in reader.connections if c.topic == '/odom']):
                stamp = reader.deserialize(raw, conn.msgtype).header.stamp
                sim.append(stamp.sec + stamp.nanosec * 1e-9)
                rec.append(ts * 1e-9)
        order = sorted(range(len(sim)), key=sim.__getitem__)
        self.sim = [sim[i] for i in order]
        self.rec = [rec[i] for i in order]

    @staticmethod
    def _interp(xs, ys, x):
        i = min(max(bisect.bisect_left(xs, x), 1), len(xs) - 1)
        x0, x1 = xs[i - 1], xs[i]
        frac = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
        return ys[i - 1] + frac * (ys[i] - ys[i - 1])

    def to_replay(self, sim_t):
        return self._interp(self.sim, self.rec, sim_t)

    def to_sim(self, replay_t):
        return self._interp(self.rec, self.sim, replay_t)


def select_runs(runs_csv):
    """First usable run per (config, nav goal): accepted, succeeded, own summary window."""
    with open(runs_csv) as f:
        rows = list(csv.DictReader(f))

    def number(row):
        return int(row['run_id'].removeprefix('run'))

    chosen = {}
    for row in sorted(rows, key=lambda r: (r['config_id'], number(r))):
        key = (row['config_id'], row['nav_goal'])
        if (key in chosen or row['window_source'] != 'monitor_summary'
                or row['summary_mismatch'] == 'True' or row['pose_reset_ok'] == 'False'
                or row['accepted'] != 'True' or row['terminal_status'] != 'succeeded'):
            continue
        chosen[key] = row
    return [chosen[k] for k in sorted(chosen)]


def run_replay(row, bags_dir, scratch, rclpy_mod):
    """Replay one bag; returns (replayed events, replay monitor summary)."""
    from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
    from rclpy.node import Node
    from unique_identifier_msgs.msg import UUID

    from safety_monitor_msgs.msg import SafetyEvent

    config_id, run_id = row['config_id'], row['run_id']
    run_dir = bags_dir / config_id / run_id
    with open(run_dir / 'monitor_summary.json') as f:
        recorded_summary = json.load(f)
    clock_map = ClockMap(run_dir / 'bag')
    t_start = clock_map.to_replay(recorded_summary['window_start_time'])
    t_end = clock_map.to_replay(recorded_summary['window_end_time'])
    terminal = STATUS_CODES.get(recorded_summary['terminal_status'], 4)
    replay_run_id = f'replay_{run_id}'

    node = Node('replay_driver', parameter_overrides=[
        rclpy_mod.parameter.Parameter('use_sim_time', value=True)])
    events = []
    node.create_subscription(
        SafetyEvent, SPI_LOG_REPLAY,
        lambda m: events.append({
            'spi_id': m.spi_id, 'zone_id': m.zone_id,
            'start': m.start_stamp.sec + m.start_stamp.nanosec * 1e-9,
            'end': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            'peak_value': m.peak_value, 'threshold': m.threshold}), 50)
    status_pub = node.create_publisher(GoalStatusArray, STATUS_TOPIC, 10)
    goal_uuid = UUID(uuid=list(os.urandom(16)))
    state = {'closes_sent': 0}

    def publish_status(code):
        msg = GoalStatusArray()
        status = GoalStatus()
        status.goal_info = GoalInfo(goal_id=goal_uuid)
        status.status = code
        msg.status_list = [status]
        status_pub.publish(msg)

    def drive():
        t = node.get_clock().now().nanoseconds * 1e-9
        if t_start <= t < t_end:
            publish_status(GoalStatus.STATUS_EXECUTING)
        elif t >= t_end and state['closes_sent'] < 5:
            publish_status(terminal)
            state['closes_sent'] += 1

    node.create_timer(0.05, drive)

    env = {**os.environ, 'ROS_DOMAIN_ID': REPLAY_DOMAIN_ID}
    monitor = subprocess.Popen(
        ['ros2', 'launch', 'safety_monitor', 'safety_monitor.launch.py',
         f'config_id:={config_id}', f'run_id:={replay_run_id}', f'bags_dir:={scratch}',
         'use_sim_time:=true', f'spi_log_topic:={SPI_LOG_REPLAY}',
         'spi_status_topic:=/spi_status_replay'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        deadline = time.time() + MONITOR_STARTUP_S
        while time.time() < deadline:
            rclpy_mod.spin_once(node, timeout_sec=0.1)
        play = subprocess.Popen(
            ['ros2', 'bag', 'play', str(run_dir / 'bag'), '--clock', '--topics', *PLAYED_TOPICS],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        while play.poll() is None:
            rclpy_mod.spin_once(node, timeout_sec=0.1)
        # Clock stops with the bag: close the window by hand if it hasn't been.
        while state['closes_sent'] < 5:
            publish_status(terminal)
            state['closes_sent'] += 1
        deadline = time.time() + SETTLE_S
        while time.time() < deadline:
            rclpy_mod.spin_once(node, timeout_sec=0.1)
    finally:
        os.killpg(monitor.pid, signal.SIGINT)
        monitor.wait(timeout=15)
        node.destroy_node()

    summary_path = scratch / config_id / replay_run_id / 'monitor_summary.json'
    replay_summary = json.load(open(summary_path)) if summary_path.exists() else {}
    for ev in events:
        ev['start'], ev['end'] = clock_map.to_sim(ev['start']), clock_map.to_sim(ev['end'])
    return events, replay_summary


def compare(row, replayed, recorded, window):
    """One row per SPI: counts and event-by-event differences."""
    out = []
    for spi_id in SPI_IDS:
        rec = sorted((e for e in recorded if e['spi_id'] == spi_id), key=lambda e: e['start'])
        rep = sorted((e for e in replayed if e['spi_id'] == spi_id), key=lambda e: e['start'])
        pairs = list(zip(rec, rep))
        out.append({
            'config_id': row['config_id'], 'run_id': row['run_id'], 'nav_goal': row['nav_goal'],
            'spi_id': spi_id, 'recorded_count': len(rec), 'replayed_count': len(rep),
            'count_match': len(rec) == len(rep),
            'max_start_diff_s': round(max((abs(a['start'] - b['start']) for a, b in pairs),
                                          default=0.0), 3),
            'max_end_diff_s': round(max((abs(a['end'] - b['end']) for a, b in pairs),
                                        default=0.0), 3),
            'max_peak_diff': round(max((abs(a['peak_value'] - b['peak_value']) for a, b in pairs),
                                       default=0.0), 4),
            'stamps_match': all(abs(a['start'] - b['start']) <= STAMP_TOLERANCE_S
                                and abs(a['end'] - b['end']) <= STAMP_TOLERANCE_S
                                for a, b in pairs),
        })
    return out


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f'Wrote {len(rows)} row(s) to {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bags-dir', default='bags', type=Path)
    parser.add_argument('--out-dir', default='bags/analysis', type=Path,
                        help='holds runs.csv (from analyze_runs.py); outputs are written here')
    args = parser.parse_args()

    os.environ['ROS_DOMAIN_ID'] = REPLAY_DOMAIN_ID
    import rclpy
    import yaml
    import analyze_runs as ar
    from safety_monitor.safety_case import SafetyCase
    from safety_monitor.zone_mask import ZoneMask  # noqa: F401  (import check only)

    with open(PKG_DIR / 'config' / 'safety_monitor.yaml') as f:
        clear_after_s = float(yaml.safe_load(f)['safety_monitor_node']['ros__parameters']
                              ['clear_after_s'])
    SafetyCase.from_file(PKG_DIR / 'config' / 'safety_case.yaml')

    runs = select_runs(args.out_dir / 'runs.csv')
    print(f'Replaying {len(runs)} bag(s): '
          + ', '.join(f"{r['config_id']}/{r['run_id']}({r['nav_goal']})" for r in runs))

    rclpy.init()
    comparison, timing_by_config, per_bag = [], {}, []
    try:
        with tempfile.TemporaryDirectory() as tmp:
            scratch = Path(tmp)
            for row in runs:
                tag = f"{row['config_id']}/{row['run_id']} ({row['nav_goal']})"
                print(f'  replaying {tag} ...', flush=True)
                replayed, replay_summary = run_replay(row, args.bags_dir, scratch, rclpy)
                if not replay_summary:
                    print(f'  !! no replay summary for {tag}: window never closed')
                    continue
                run_dir = args.bags_dir / row['config_id'] / row['run_id']
                recorded_summary = ar.load_json(run_dir / 'monitor_summary.json')
                _, _, _, raw_events = ar.read_bag(run_dir / 'bag')
                window = (recorded_summary['window_start_time'],
                          recorded_summary['window_end_time'])
                recorded = ar.correct_events(raw_events, recorded_summary, clear_after_s, window)
                comparison += compare(row, replayed, recorded, window)
                timing = replay_summary.get('processing_time_ms') or {}
                timing_by_config.setdefault(row['config_id'], []).append(timing)
                for check in ('tick', 'spi4_callback'):
                    per_bag.append({'config_id': row['config_id'], 'run_id': row['run_id'],
                                    'nav_goal': row['nav_goal'], 'check': check,
                                    **{k: ar.r(v) for k, v in (timing.get(check) or {}).items()}})
    finally:
        rclpy.shutdown()

    if not comparison:
        print('Nothing replayed.')
        return
    args.out_dir.mkdir(parents=True, exist_ok=True)
    eff = ar.efficiency_rows(timing_by_config)
    for e in eff:
        e['source'] = 'replay'
    write_csv(args.out_dir / 'efficiency.csv', eff)
    write_csv(args.out_dir / 'efficiency_per_bag.csv', per_bag)
    write_csv(args.out_dir / 'replay_comparison.csv', comparison)
    bad = [c for c in comparison if not (c['count_match'] and c['stamps_match'])]
    print(f'{len(comparison) - len(bad)}/{len(comparison)} (bag, SPI) pairs agree '
          f'on count and stamps (tolerance {STAMP_TOLERANCE_S} s)')


if __name__ == '__main__':
    main()
