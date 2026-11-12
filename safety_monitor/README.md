# safety_monitor

Passive, observe-only measurement framework for comparing four Nav2
configurations (`base`, `native_keepout`, `native_speed`, `external`) under
identical conditions, for the thesis "A Retrofit Methodology for Continuous
Safety Assurance in Autonomous Systems" (RQ1-RQ3).

It never publishes anything the system under test consumes (only
`/spi_log` and `/spi_status`), and classifies zones from its own read of
`maps/zone_map.pgm` rather than from any config-specific topic, so the same
monitor observes all four configurations unmodified.

## Packages

- `safety_monitor_msgs`: the `SafetyEvent` message.
- `safety_monitor`: the monitor node, launch file, and run tooling.

## Running one experiment run

Terminal 1 -- bring up the configuration under test, always from the same
start pose. Every sim launch file already defaults `x_pose`/`y_pose` to
`0.0`/`0.0` -- that's also the origin cell in RViz, which makes "2D Pose
Estimate" easy to place accurately by eye, so no override is needed:

```bash
ros2 launch my_robot_sim sim_baseline.launch.py
# or sim_keepout.launch.py / sim_speed.launch.py / drc/sim.launch.py
```

Terminal 2 -- the monitor. `config_id`/`run_id` here are just the
*initial* values: leave this running for the whole session (across
multiple goals/runs) rather than restarting it each time -- `send_goal.py`
below pushes its own `config_id`/`run_id` into this already-running node
via its `/set_parameters` service right before every goal, so each run's
`monitor_summary.json` still lands under the correct `run_id` without a
restart:

```bash
ros2 launch safety_monitor safety_monitor.launch.py config_id:=base run_id:=run1
```

Terminal 3 -- start recording for this run, then send a goal (repeat
terminals 3 and 4 for each subsequent run, with a new `run_id`, without
touching terminal 1 or 2):

```bash
ros2 run safety_monitor record_run.sh base run1
```

Terminal 4:

```bash
gz service -s /world/my_room/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'name: "turtlebot3_waffle" position: {x: 0.0, y: 0.0, z: 0.0} orientation: {w: 1.0}'

ros2 run safety_monitor send_goal.py --ros-args -p config_id:=base -p run_id:=run1 -p goal:=goal1 -p reset_pose:=true

ros2 run safety_monitor send_goal.py --ros-args -p config_id:=base -p run_id:=run1 -p goal:=goal2 -p reset_pose:=false

ros2 run safety_monitor send_goal.py --ros-args -p config_id:=base -p run_id:=run1 -p goal:=goal3 -p reset_pose:=false

```

### One run = one goal; one circuit = all three goals

`/initialpose` only resets AMCL's *belief* about where the robot is -- it
does not move the robot in Gazebo. So resetting to the start pose before
every single goal would leave AMCL's belief out of sync with the robot's
real simulated position (which is still wherever the previous goal ended),
breaking localization. `send_goal.py` therefore also teleports the robot
in Gazebo itself (`gz service /world/my_room/set_pose`) whenever it resets
to the start pose -- both only happen when `reset_pose` is `true` (the
default).

Run a **circuit** of all three goals per repetition, resetting to (0, 0)
only before the first goal of each circuit -- pass `reset_pose:=false` for
goal2 and goal3 so they continue from wherever the previous goal in the
same circuit actually left the robot:

```bash
ros2 run safety_monitor record_run.sh base run1
ros2 run safety_monitor send_goal.py --ros-args -p config_id:=base -p run_id:=run1 -p goal:=goal1 -p reset_pose:=true
# (stop/restart the recorder between each goal as usual)
ros2 run safety_monitor record_run.sh base run2
ros2 run safety_monitor send_goal.py --ros-args -p config_id:=base -p run_id:=run2 -p goal:=goal2 -p reset_pose:=false
ros2 run safety_monitor record_run.sh base run3
ros2 run safety_monitor send_goal.py --ros-args -p config_id:=base -p run_id:=run3 -p goal:=goal3 -p reset_pose:=false
# next circuit starts over with reset_pose:=true on goal1 again
```

Each goal still gets its own `run_id`, `bag/`, `outcome.json` and
`monitor_summary.json` -- the circuit only changes the robot's physical
starting point for goal2/goal3, not the run/analysis granularity.

To manually move the robot back to the start pose yourself (e.g. after an
aborted run leaves it stranded), with a sim running:

```bash
gz service -s /world/my_room/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'name: "turtlebot3_waffle" position: {x: 0.0, y: 0.0, z: 0.0} orientation: {w: 1.0}'
```

Manually-sent goals (RViz's "Nav2 Goal" tool, no `send_goal.py` involved)
still open/close the SPI window and produce a `monitor_summary.json` --
but since nothing calls `_sync_monitor_run_id`, it's written under
whichever `run_id` the monitor was last given. Use
`ros2 param set /safety_monitor_node run_id <name>` yourself beforehand if
you want a manual goal's summary kept separate from the next one.

`goal` is `goal1` ((0, 4.2), speed zone), `goal2` ((-6, -3), free space,
no zone crossed) or `goal3` ((-9, 0), keepout zone -- the straight path
from the start pose runs directly through it, x in roughly [-7.5, -4.0]
at y=0). `send_goal.py` publishes `/initialpose` for the known start pose
before sending the goal, and writes
`bags/base/run1/outcome.json` with the accept/reject result and, once
available, the terminal status -- this is the only place a *rejected* goal
is recorded, since it never appears on the action status topic the monitor
itself watches.

The keepout zone sits far enough from the (0, 0) start pose (nearest edge
at x=-4.0, i.e. 4m away) that `zone_detector.py`'s default `detection_range`
(3.0m) never reveals it before the robot moves -- in the `external`
config, the zone genuinely has to be discovered during the run to goal3,
not already known from the start pose.

Stop the recording (Ctrl-C in terminal 3) once `send_goal.py` exits. A run
leaves three files behind:

```
bags/base/run1/
  bag/                  # ros2 bag, all topics listed in record_run.sh
  outcome.json          # written by send_goal.py
  monitor_summary.json  # written by safety_monitor_node on goal result
```

For the thesis, repeat this 5-10 times per configuration per goal.

### Automating terminals 3 and 4: `run_campaign.py`

Once terminal 1 (sim) and terminal 2 (monitor, for the right `config_id`)
are up, `run_campaign.py` drives the circuit loop above for you -- starting
and stopping `record_run.sh` and calling `send_goal.py` with the right
`reset_pose` for every goal in every repetition, auto-detecting the next
free run number under `bags/<config_id>/` so re-running after a partial
campaign never clobbers existing runs:

```bash
ros2 run safety_monitor run_campaign.py base --reps 10
ros2 run safety_monitor run_campaign.py external --reps 10 --goals goal1 goal3
ros2 run safety_monitor run_campaign.py base --reps 1 --dry-run   # preview only
```

This is what was actually used to collect the thesis dataset. The manual
terminal 3/4 steps above are what it does under the hood -- worth knowing
if a single run needs to be redone or debugged by hand.

## Parameters

- `config/safety_monitor.yaml`: monitor runtime parameters (frames, pose
  source, `tick_hz`, debounce `clear_after_s`).
- `config/safety_case.yaml`: the machine-readable safety argument and the
  **single source of truth for SPI thresholds and the SPI -> goal mapping**.
  The monitor reads its thresholds from here and puts the goal id into every
  `SafetyEvent.claim_id` (`""` for the informational SPI-4).

`config_id`, `run_id`, `zone_map_yaml`, `safety_case_yaml`, `bags_dir` and
`pose_source` are also exposed as `safety_monitor.launch.py` arguments.

## Safety argument (`config/safety_case.yaml`)

| Goal | Claim | Hazard | Evidence | Exposure (min) |
|---|---|---|---|---|
| G1 | Acceptably safe w.r.t. restricted zones | H0 | supported by G2-G4 | -- |
| G2 | Zone speed limits respected | H1 | SPI-1 (> 0.2 m/s in speed zone) | distance in a speed zone (0.5 m) |
| G3 | Keepout zones never entered | H2 | SPI-2 (inside keepout) | distance within 1.0 m of keepout (0.5 m) |
| G4 | Robot keeps a safety margin to keepout zones | H3 | SPI-3 keepout margin, map-based (< 0.3 m) | time within 1.5 m of keepout (1.0 s) |
| -- | informational | -- | SPI-4 (intervention) | whole run |

Each evidence entry also has a `tolerance` (applied offline only: an event
counts against its goal only if it exceeds the threshold by more than this;
`0.0` = every event counts).

## Event timestamps

A `SafetyEvent` spans the **first to the last violating sample** of one
debounced episode: violations separated by gaps shorter than `clear_after_s`
(0.3 s) are merged into one event. `start_stamp` is the first violating
sample, `header.stamp` the last.

Runs recorded before this convention stamped the end at *publish* time,
i.e. `clear_after_s` after the last violating sample. `analyze_runs.py`
recognises them (no `event_stamp_convention` in `monitor_summary.json`) and
shifts both stamps back by `clear_after_s` (read from
`config/safety_monitor.yaml`). Events that were flushed by the goal window
closing (published within 20 ms of the window end) are not shifted, since
the debounce delay never elapsed for them. Checked against the `/odom` trace,
corrected stamps land within one or two ticks (≤ 0.15 s) of the true
first/last violating sample. Event durations are unaffected either way.

## Offline analysis

```bash
python3 analysis/analyze_runs.py --bags-dir bags --out bags/analysis/runs.csv
```

Reads every `bags/<config_id>/<run_id>/` directory and writes, next to
`--out`:

| File | Content |
|---|---|
| `runs.csv` | one row per run: outcome, mission cost, per-SPI count / duration / peak / distance driven during events / exposure, in-zone speed cross-check, monitor processing time |
| `events.csv` | one row per `/spi_log` event, with corrected stamps and distance driven |
| `claim_status.csv` | run × goal: `supported` / `challenged` / `no_evidence` plus count, peak, event duration ÷ exposure time, events per metre of exposure |
| `claim_summary.csv` | per config (all nav goals, and per nav goal): how many runs support / challenge / give no evidence for each goal |
| `filtering_stages.csv` | messages and bytes at (a) all recorded raw topics, (b) the topics the SPIs use, (c) `/spi_log` events, (d) claim-status rows, with reduction factors. `/scan` was not recorded in the existing bags (missing from `record_run.sh`'s topic list, since fixed), so stage (a) is a lower bound (`note` column) |
| `efficiency.csv` | monitor processing time per check cycle (mean, p95, max) per config; from `monitor_summary.json` of live runs, or written by `replay_bags.py` (`source` column) |
| `spi1_overspeed.csv` | per config, nav goal and SPI-1 tolerance: time (s) and distance (m) driven in a speed zone above limit + tolerance, mean ± std over runs that entered a speed zone (main SPI-1 comparison; from `/odom`, independent of the monitor's event fragmentation); peak speed as secondary columns |
| `tolerance_sensitivity.csv` | claim-status counts per config / nav goal / goal with the SPI-1 tolerance at each value of `--spi1-tolerances` (default 0 and 0.02 m/s) |
| `g3_challenged.csv`, `g4_challenged.csv` | one row per event challenging G3 / G4: nav goal, duration, penetration depth (G3) or clearance shortfall (G4), map-frame position at the peak; `within_grid_resolution` flags SPI-2 penetrations ≤ the map resolution (0.05 m). Status is not changed by the flag |
| `source_check.csv` | per run: where its events and window came from (own bag `/spi_log` vs `monitor_summary.json`), event counts from both |
| `gsn_<config>.dot` / `.svg` | GSN fragment, goals coloured by claim status (`.svg` only if Graphviz is installed; `--no-gsn` skips this) |

Everything is clipped to the monitor's active-goal window, using message
header stamps (sim time, the same clock as `monitor_summary.json`).

**Exposure** (how much opportunity an SPI had to fire) is recomputed offline
from `/odom` + `/tf` (`map->odom`) + `zone_map.pgm`, so old and new runs
use the same source. Segments longer than 0.5 s (gaps) or faster than 2 m/s
(teleports, AMCL re-localisation) are not counted.

**Claim status** per run and goal:
- `challenged`: at least one event counting against the goal
- `supported`: zero events **and** exposure ≥ `min_exposure`
- `no_evidence`: zero events but too little exposure to support the claim

A goal with sub-goals (G1) is `challenged` if any child is, `supported` only
if all children are, otherwise `no_evidence`. G4's exposure is the time spent
within 1.5 m of a keepout cell (`within_m` in `safety_case.yaml`; map-based
distance, the signal SPI-3 itself uses). 1.5 m exceeds `inflation_radius`
(0.7 m) + `robot_radius` (0.22 m), so every run whose planner had to work
around a keepout zone counts as exposed. A run that never gets that close
gives `no_evidence` for G4 rather than a vacuous `supported`.

**Filtering stage (b)** lists `/navigate_to_pose/_action/status`, but that is
a hidden topic. The existing bags were recorded without it, so it contributes
0 messages there; `record_run.sh` now passes `--include-hidden-topics`, so
future bags hold it.

**Processing time** comes from `monitor_summary.json` (`processing_time_ms`:
the 20 Hz tick running SPI-1..3, and the `/cmd_vel` callback running SPI-4).
Runs recorded before this was added have no timing; `efficiency.csv` reports
how many runs had data (`n_runs_with_timing`).

`runs.csv` also keeps the independent in-zone speed cross-check
(`speed_zone_median_mps`, `speed_zone_mean_mps`, `speed_zone_entry_peak_mps`),
which tells "the robot was actually faster" apart from "the event count
just fragmented differently" without trusting the monitor's own SPI-1.
Pass `--zone-map-pgm`/`--zone-map-yaml` if the map moved.

**Frozen results** (tag `analysis-freeze`): a copy of every output above, plus
the replay outputs below, is versioned in `analysis/results/`. The bags themselves
are not in the repository.

## Replaying bags through the monitor (`analysis/replay_bags.py`)

Efficiency and a check of the monitor itself without new sim runs: one bag per
config and nav goal is replayed with `ros2 bag play --clock` while the real
`safety_monitor.launch.py` runs (`use_sim_time:=true`, `run_id:=replay_<run>`,
output remapped to `/spi_log_replay` via the `spi_log_topic` launch argument).

```bash
python3 analysis/replay_bags.py --bags-dir bags --out-dir bags/analysis   # needs runs.csv
```

- Runs on its own `ROS_DOMAIN_ID` (87), so a running sim is not disturbed.
- `ros2 bag play --clock` publishes the time each message was *recorded*
  (wall time), not the sim time in the header stamps. The script maps between
  the two via `/odom` (`ClockMap`).
- The bags have no `/navigate_to_pose/_action/status` (hidden topic), so the
  script opens and closes the monitor's goal window with a synthetic status
  message at the recorded window start/end. Bags recorded with
  `--include-hidden-topics` (as `record_run.sh` now does) hold the real one.
- Outputs `efficiency.csv`, `efficiency_per_bag.csv` and
  `replay_comparison.csv` (events per SPI, replayed vs recorded: count, start /
  end / peak differences).

## Known limitation: run_id sync and mismatched summaries

`send_goal.py` pushes `config_id`/`run_id` into the running monitor before
every goal. Older versions sent the goal even when that push failed. The
monitor then opened the new goal's window under the *previous* `run_id`,
overwrote the previous run's `monitor_summary.json`, and left the new run
without one (seen in `external/run15`-`16` and `run25`-`26`). `send_goal.py`
now retries the push three times and, if it still fails, does not send the
goal and records `error_msg: "monitor run_id sync failed; goal not sent"`
in `outcome.json`.

For the existing data, `analyze_runs.py` detects a summary whose window lies
outside the bag's own `/odom` time span (`summary_mismatch` in `runs.csv`),
ignores it, and uses the span in which the robot moves as the window
(`window_source: bag_motion`). Those runs have no `time_to_goal_s` /
`path_length_m` from the monitor.

## Known limitation: odometry drift across a long circuit-mode session

Teleporting the robot in Gazebo (`reset_pose:=true`) changes its true
position but never touches the diff-drive plugin's own odometry
integrator, which purely dead-reckons from real wheel rotation since the
sim started. Every circuit genuinely drives out to a goal and gets
teleported back instead of actually driving back, so `odom`'s drift from
true position never resets and keeps growing, circuit after circuit. That
makes the single `/initialpose` correction AMCL has to compute at the
start of each circuit progressively larger -- and less reliable -- the
longer a campaign runs. `send_goal.py`'s `_verify_reset` retries the whole
teleport+`/initialpose`+costmap-clear sequence (up to 3 times) and checks
the robot actually converged to within 0.3m of the start pose afterward,
logging the result as `pose_reset_ok`/`pose_reset_distance_m` in
`outcome.json` (and surfaced in `analyze_runs.py`'s CSV). **Filter out any
run where `pose_reset_ok` is `false` before comparing configurations** --
it started from an uncontrolled position, so its SPI/path/duration numbers
aren't comparable to a correctly-reset run. If this turns out to be
common in your data, restarting the whole sim+monitor session periodically
(every N circuits) resets the odometry integrator along with everything
else and is the more thorough fix.

## Known limitation: goal2 is not a clean zone-free control

Originally, `goal2` ((-6, -3)) was meant to cross neither zone -- a
control condition for comparing against `goal1` (speed zone) and `goal3`
(keepout zone). After the keepout zone was relocated (to stay outside
`zone_detector.py`'s detection range from the start pose), the straight
line from `(0, 0)` to `(-6, -3)` now passes through `(-4, -2)` -- exactly
the keepout zone's corner -- and the line from `goal1`'s end pose
`(0, 4.2)` to `goal2` cuts through it more deeply still. Treat `goal2` as
crossing the keepout zone too (like `goal3`) rather than as a zone-free
control when writing up results, or relocate the keepout zone again to
clear both line segments if a true zone-free control matters for the
analysis.

For `base`/`external` (no keepout-aware local planning) this reliably
produces a real `SPI-2` event every `goal2` run. For `native_keepout`
after the local-costmap fix below, the global plan routes around the
corner closely enough that most runs now show `SPI-2: 0` -- only an
occasional single-grid-cell graze (~1 in 10 runs) remains, not a
deterministic violation every time.

## `native_keepout`: KeepoutFilter must be on the local costmap, not just global

The stock Nav2 keepout-zones pattern wires `KeepoutFilter` only into
`global_costmap` (`config/nav2_params_keepout.yaml`), which only keeps the
zone out of the *planned* path -- the local costmap (and therefore the
`MPPIController`'s own collision-cost critic, which samples trajectories
against the local costmap in real time) has no idea the zone exists.
Empirically this produced a consistent, reproducible single-grid-cell
(0.05 m, matching the mask's resolution) graze of the keepout corner on
every `goal2` run: the global plan threads right along the lethal
boundary, and nothing downstream of planning corrects it.

Mirroring the same `filters: ["keepout_filter"]` + `keepout_filter` block
into `local_costmap` (pointing at the same `/costmap_filter_info` topic --
no launch file changes needed) fixes this: MPPI's `collision_cost`
critic now sees the zone in real time and steers clear of it, dropping the
violation rate from 10/10 to roughly 1/10 `goal2` runs (the residual case
observed so far was the very first goal right after a sim/monitor restart,
before TF/costmaps had fully settled -- plausibly avoidable by excluding
the first circuit after any restart, though not yet confirmed with enough
samples to be sure it isn't also just MPPI's sampling-based optimization
occasionally finding a boundary-grazing trajectory on its own). This is
the config now used for all `native_keepout` data in the thesis dataset --
worth reporting as a finding in its own right, since it's not how the
stock Nav2 tutorial configures this filter.

## Known limitation: pose source

`pose_source` defaults to `tf` (`map` -> `base_footprint`, via the same
pattern `pose_adapter.py` uses). The world does load Gazebo's
`SceneBroadcaster` plugin, so true ground-truth pose exists in principle
at the Gazebo level (`/world/my_room/pose/info`), and the node already
accepts a `pose_source:=gt` parameter plus a `gt_pose_topic` for a bridged
`geometry_msgs/PoseStamped` -- but wiring an actual `ros_gz_bridge` for it
needs a live sim run to confirm the exact topic and entity names, which
wasn't done in the time available. `tf` is what every run in this thesis
actually uses.
