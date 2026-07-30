# K1 Path Following — Record, Extract, Run

Full workflow: teach a path by walking/moving the robot through it while recording, extract the
path as waypoints, then run `path_follower` to replay it.

## 1. Record data

Records `/odom` (plus `/imu/data`, `/joint_states`, `/tf`, `/tf_static`, and the head combine
camera) to a rosbag.

```bash
./record_data.sh RUN_ID
```

- `RUN_ID` is a name for this run, e.g. `TEST_003` (optional — defaults to `FIELD_<timestamp>` if omitted).
- Output goes to `~/k1_field_data/RUN_ID/rosbag` (override the root with `K1_DATA_ROOT=/path`).
- Recording continues until you press Ctrl+C, or hits a safety stop (disk/RAM low).
- To auto-stop after N seconds instead of pressing Ctrl+C:
  ```bash
  timeout N ./record_data.sh RUN_ID
  ```
- While it's recording, physically walk/move the robot along the path you want it to learn.

## 2. Extract waypoints from the recording

```bash
python3 extract_odometry_map.py \
  --bag ~/k1_field_data/RUN_ID/rosbag \
  --out-dir ~/Workspace/RUN_ID_map \
  --spacing-m 0.3
```

- `--bag` points at the rosbag folder (the one containing `metadata.yaml`).
- `--out-dir` is where the outputs go — creates `waypoints.csv` (what `path_follower` reads),
  plus `odom_dense.csv`, `summary.json`, and some plots.
- `--spacing-m 0.3` sets how far apart (in meters) waypoints get placed along the path (default
  is 0.5 if omitted — 0.3 is what this project has been using).

Sanity-check the result before trusting it:
```bash
cat RUN_ID_map/waypoints.csv
```
Look for a `start` row and an `end` row with a sane `cumdist_m` on the last row (roughly matching
how far you actually walked).

## 3. Build path_follower (only needed once, or after editing the source)

```bash
source /opt/ros/humble/setup.bash
source /opt/booster/BoosterRos2Interface/install/setup.bash
source /opt/booster/ros2/booster_msgs/share/booster_msgs/local_setup.bash

cd /home/booster
colcon build --packages-select path_follower_pkg --base-paths ~/Workspace/path_follower_pkg
```

## 4. Run path_follower

Source the environment (needed every new terminal):
```bash
source /opt/ros/humble/setup.bash
source /opt/booster/BoosterRos2Interface/install/setup.bash
source /opt/booster/ros2/booster_msgs/share/booster_msgs/local_setup.bash
source /home/booster/install/setup.bash
```

**Dry run first** (default — logs what it would do, sends nothing to the robot):
```bash
ros2 run path_follower_pkg path_follower --ros-args \
  -p waypoints_csv:=~/Workspace/RUN_ID_map/waypoints.csv
```

**Stand the robot near where the path started**, then run for real:
```bash
ros2 run path_follower_pkg path_follower --ros-args \
  -p waypoints_csv:=~/Workspace/RUN_ID_map/waypoints.csv \
  -p send:=true
```

To auto-stop it after N seconds as a safety cutoff (sends the same signal as Ctrl+C, which this
program shuts down cleanly on — but it does NOT stop the robot's motion on exit, so only use this
if the robot is expected to have already reached the goal by then):
```bash
timeout -s INT N ros2 run path_follower_pkg path_follower --ros-args \
  -p waypoints_csv:=~/Workspace/RUN_ID_map/waypoints.csv \
  -p send:=true
```

### What happens on startup
1. Waits 2s for `/odom` to deliver a few real readings.
2. Calibrates the loaded waypoints to the robot's *current* position and heading (shifts +
   rotates the whole path) — this assumes the robot is standing close to physically where it was
   when the path was recorded. If you see `[FAIL] No /odom received...`, just re-run the same
   command — this has been an intermittent startup race, not a real failure.
3. Sends `ChangeMode(kWalking)` and confirms it via `/robot_states` before moving at all.
4. Runs the Pure Pursuit control loop at 20Hz until the goal is reached, then stops and exits the
   control loop cleanly.

### Useful tunable parameters (all overridable with `-p name:=value`)
| Parameter | Default | What it does |
|---|---|---|
| `waypoints_csv` | (required) | Path to the waypoints CSV from step 2 |
| `send` | `false` | `true` actually moves the robot; `false` only logs |
| `lookahead_m` | `0.3` | How far ahead to pick the target waypoint |
| `v_nom` | `0.25` | Nominal forward walking speed (m/s) |
| `v_max` | `0.35` | Max forward speed |
| `w_max` | `0.9` | Max turn rate (rad/s) |
| `goal_tol` | `0.20` | How close (m) to the final waypoint counts as "arrived" |
| `control_hz` | `20.0` | Control loop rate |
