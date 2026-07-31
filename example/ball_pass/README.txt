# Ball-Pass — Setup and Run

Vision-driven K1 behavior: detects a ball and a person via the head camera, and kicks/passes the
ball toward the person. Uses the Booster SDK for locomotion/kick (not ROS2) and a separate ROS2
vision pipeline for detection, bridged between the two processes via a shared-memory file (see
"Why two processes" below).

## Files in this repo

- `pass_ball_to_person_startup.sh` — brings up the vision pipeline (camera driver, detection, bridge).
- `build_no_ros/visual_pass_ball_to_person.cpp` — the actual behavior (SDK-based, reads detections,
  commands locomotion/kick).
- `build_no_ros/vision_config/vision.yaml` — our own override of `robocup_demo`'s vision config
  (topic names matching this robot's actual camera driver).
- `vision_ros_bridge_pkg/` — ROS2 package:
  - `src/vision_bridge_node.cpp` — **required**. Subscribes to `vision_node`'s detections, writes
    them to `/dev/shm/booster_vision_bridge.txt` for the non-ROS2 behavior binary to read.
  - `src/vision_stream_server.cpp` — optional. Serves an annotated live camera feed (detection
    boxes/labels/distance) as MJPEG-over-HTTP on port 8090, for debugging/viewing only.
  - `src/raw_stream_server.cpp`, `src/depth_recorder.cpp` — unrelated recording tools, not needed
    for this behavior.
- `CMakeLists.txt` (top-level, in `~/Workspace/`) — builds `visual_pass_ball_to_person` and the
  other SDK-only (non-ROS2) executables. `build_no_ros/` is the out-of-source build directory for
  this file (no `CMakeLists.txt` of its own).

## External dependency (not part of this repo)

- **`robocup_demo`** — Booster's own vision/detection ROS2 package (`vision_node`). This must
  already be cloned and built separately at `~/Workspace/booster_demo/robocup_demo` (needs its own
  `install/setup.bash`) - not part of this repo, refer to Booster's own instructions for building it.

## Why two processes

ROS2 Humble's `rmw_fastrtps_cpp` needs `libfastrtps.so.2.6`, while the Booster SDK needs
`libfastrtps.so.2.13` - two ABI-incompatible versions that crash if both create a DDS participant
in the same process. So the vision side (ROS2: `vision_node`, `vision_bridge_node`) and the
behavior side (SDK: `visual_pass_ball_to_person`) run as separate processes, bridged through a
plain file (`/dev/shm/booster_vision_bridge.txt`) rather than any ROS2/DDS mechanism.

## Prerequisites

- ROS2 Humble installed.
- `robocup_demo` built (see above).
- `vision_ros_bridge_pkg` and the top-level SDK build both built (steps below).
- `sshpass` installed (`sudo apt install sshpass` if missing).
- Network access to the camera board (`192.168.127.10`, user `root` - credentials are in
  `pass_ball_to_person_startup.sh`).

## Build

**`vision_ros_bridge_pkg`** (ROS2 package):
```bash
source /opt/ros/humble/setup.bash
cd ~/Workspace/vision_ros_bridge_pkg
colcon build
```

**`visual_pass_ball_to_person`** (SDK-only, non-ROS2):
```bash
cd ~/Workspace/build_no_ros
cmake ..
make visual_pass_ball_to_person
```

## Run

**Step 1 — bring up the vision pipeline** (from `~/Workspace/`):
```bash
./pass_ball_to_person_startup.sh
```
This will, in order:
1. Kill any leftover vision processes from a previous run (safe to re-run any time).
2. Switch the camera board's driver into `cam_rectify_stream` mode over SSH (needed after every
   camera board reboot - only `cam_rectify_stream` exposes the TCP port the camera receiver needs).
3. Launch `vision_node` (from `robocup_demo`) with our `vision_config` override.
4. Launch `vision_bridge_node` (writes detections to `/dev/shm/booster_vision_bridge.txt`).
5. Launch `vision_stream_server` (optional debug feed on port 8090).

All of these run in the background; logs go to `/tmp/pass_ball_to_person_*.log`.

**Step 2 — run the actual behavior**, in its own terminal:
```bash
cd ~/Workspace/build_no_ros
./visual_pass_ball_to_person lo
```
(`lo` is the network interface, matching the other SDK tools in this project.)

## Useful while running

- Watch live detections: `watch -n0.5 cat /dev/shm/booster_vision_bridge.txt`
- View the annotated camera feed from a laptop:
  ```bash
  ssh -L 8090:localhost:8090 <user>@<robot-ip>
  ```
  then open `http://localhost:8090` in a browser.

## Stopping the vision pipeline

`ros2 launch`/`ros2 run` spawn the real node binaries as child processes, so killing just the
`ros2` wrapper PID leaves them running as orphans. Use pattern-matched kills instead (also what
`pass_ball_to_person_startup.sh` does automatically at the start of every run):
```bash
pkill -9 -f "vision/lib/vision/vision_node"
pkill -9 -f "ros2 launch vision launch.py"
pkill -9 -f "vision_ros_bridge_pkg/lib/vision_ros_bridge_pkg/vision_bridge_node"
pkill -9 -f "ros2 run vision_ros_bridge_pkg vision_bridge_node"
pkill -9 -f "vision_ros_bridge_pkg/lib/vision_ros_bridge_pkg/vision_stream_server"
pkill -9 -f "ros2 run vision_ros_bridge_pkg vision_stream_server"
```
