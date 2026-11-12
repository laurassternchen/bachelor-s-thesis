#!/usr/bin/env bash
# Records one experiment run's topics into bags/<config_id>/<run_id>/bag/.
#
# Usage: record_run.sh <config_id> <run_id> [bags_dir]
#
# A superset of topics is listed so the SAME command works for all four
# configurations under test -- `ros2 bag record` simply waits for any
# topic that doesn't exist in a given config rather than failing.
# --include-hidden-topics is needed for /navigate_to_pose/_action/status:
# topics with a leading-underscore name segment are hidden and skipped otherwise.
set -euo pipefail

if [[ $# -lt 2 ]]; then
    echo "usage: $0 <config_id> <run_id> [bags_dir]" >&2
    exit 1
fi

CONFIG_ID="$1"
RUN_ID="$2"
BAGS_DIR="${3:-bags}"
OUT_DIR="${BAGS_DIR}/${CONFIG_ID}/${RUN_ID}"
BAG_DIR="${OUT_DIR}/bag"

if [[ -e "${BAG_DIR}" ]]; then
    echo "error: ${BAG_DIR} already exists -- ros2 bag record refuses to overwrite it" >&2
    exit 1
fi
mkdir -p "${OUT_DIR}"

TOPICS=(
    /spi_log
    /spi_status
    /odom
    /scan
    /tf
    /tf_static
    /cmd_vel
    /cmd_vel_smoothed
    /cmd_vel_safe
    /speed_limit
    /zone_truth
    /detected_zones
    /navigate_to_pose/_action/status
)

exec ros2 bag record --include-hidden-topics -o "${BAG_DIR}" "${TOPICS[@]}"
