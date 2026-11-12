#!/usr/bin/env python3
"""Drives a full measurement campaign for one already-running configuration:
repeats a circuit of goals (goal1 -> goal2 -> goal3 by default, reset_pose
only on the first of each circuit) for a number of repetitions, starting
and stopping record_run.sh and calling send_goal.py for each run so none
of it needs to be typed by hand.

Only automates terminals 3 and 4 -- the sim (terminal 1) and the monitor
(terminal 2) must already be running for the config_id being tested,
exactly as in the manual workflow in README.md.

Auto-detects the next free run number under bags/<config_id>/ so it's
safe to re-run after a partial campaign without clobbering existing runs.

Usage:
    ros2 run safety_monitor run_campaign.py base --reps 10
    ros2 run safety_monitor run_campaign.py external --reps 5 --goals goal1 goal3
    ros2 run safety_monitor run_campaign.py base --reps 1 --dry-run
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import time

DEFAULT_GOALS = ['goal1', 'goal2', 'goal3']
RECORDER_STARTUP_S = 2.0  # time for `ros2 bag record` to attach before the goal is sent
RECORDER_SHUTDOWN_TIMEOUT_S = 15.0

# Mirrors record_run.sh's own TOPICS list -- kept in sync by hand. Recording
# is invoked directly here (not via `ros2 run ... record_run.sh`) so the
# Popen handle below is the actual `ros2 bag record` process, not a launcher
# wrapper that may exit before the real recorder does, leaving signals sent
# to this handle with nothing left alive underneath to receive them.
TOPICS = [
    '/spi_log', '/spi_status', '/odom', '/tf', '/tf_static', '/cmd_vel',
    '/cmd_vel_smoothed', '/cmd_vel_safe', '/speed_limit', '/zone_truth',
    '/detected_zones', '/navigate_to_pose/_action/status',
]


def next_run_number(bags_dir, config_id):
    config_dir = os.path.join(bags_dir, config_id)
    if not os.path.isdir(config_dir):
        return 1
    existing = []
    for d in os.listdir(config_dir):
        m = re.fullmatch(r'run(\d+)', d)
        if m:
            existing.append(int(m.group(1)))
    return max(existing, default=0) + 1


def run_one(config_id, run_id, goal, reset_pose, bags_dir, max_goal_s):
    out_dir = os.path.join(bags_dir, config_id, run_id)
    bag_dir = os.path.join(out_dir, 'bag')
    os.makedirs(out_dir, exist_ok=True)

    recorder = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_dir, *TOPICS])
    time.sleep(RECORDER_STARTUP_S)

    try:
        result = subprocess.run([
            'ros2', 'run', 'safety_monitor', 'send_goal.py', '--ros-args',
            '-p', f'config_id:={config_id}',
            '-p', f'run_id:={run_id}',
            '-p', f'goal:={goal}',
            '-p', f'reset_pose:={"true" if reset_pose else "false"}',
            '-p', f'bags_dir:={bags_dir}',
            '-p', f'max_goal_s:={max_goal_s}',
        ])
        ok = result.returncode == 0
    finally:
        # SIGINT, not .terminate()/SIGTERM: `ros2 bag record` only writes
        # metadata.yaml (making the bag readable) on a clean SIGINT shutdown.
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=RECORDER_SHUTDOWN_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            print(f'  warning: recorder for {run_id} did not exit within '
                  f'{RECORDER_SHUTDOWN_TIMEOUT_S}s, sending SIGTERM', file=sys.stderr)
            recorder.terminate()
            recorder.wait(timeout=5.0)

    if not os.path.exists(os.path.join(bag_dir, 'metadata.yaml')):
        print(f'  warning: {bag_dir} has no metadata.yaml -- bag may not be finalized',
              file=sys.stderr)
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config_id')
    parser.add_argument('--reps', type=int, default=10, help='circuits to run')
    parser.add_argument('--goals', nargs='+', default=DEFAULT_GOALS)
    parser.add_argument('--bags-dir', default='bags')
    parser.add_argument('--max-goal-s', type=float, default=120.0)
    parser.add_argument('--dry-run', action='store_true',
                         help='print the planned run_id/goal/reset_pose sequence '
                              'without touching ROS/Gazebo at all')
    args = parser.parse_args()

    run_number = next_run_number(args.bags_dir, args.config_id)
    print(f'Starting at run{run_number}, {args.reps} circuit(s) of {args.goals} '
          f'for config_id={args.config_id}{" [dry-run]" if args.dry_run else ""}')

    for circuit in range(1, args.reps + 1):
        for i, goal in enumerate(args.goals):
            run_id = f'run{run_number}'
            reset_pose = (i == 0)
            print(f'[circuit {circuit}/{args.reps}] {run_id}: goal={goal} '
                  f'reset_pose={reset_pose}')
            if not args.dry_run:
                try:
                    ok = run_one(args.config_id, run_id, goal, reset_pose,
                                 args.bags_dir, args.max_goal_s)
                except KeyboardInterrupt:
                    print('\nInterrupted -- the in-progress recorder was already '
                          'signaled to stop cleanly', file=sys.stderr)
                    sys.exit(1)
                if not ok:
                    print(f'  send_goal.py exited non-zero for {run_id} -- '
                          f'continuing to the next run anyway', file=sys.stderr)
            run_number += 1

    print(f'Campaign done: {run_number - 1} run(s) written under '
          f'{args.bags_dir}/{args.config_id}/')


if __name__ == '__main__':
    main()
