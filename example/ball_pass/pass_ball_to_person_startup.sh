#!/bin/bash
# Brings up the vision pipeline that visual_pass_ball_to_person depends on:
#   1. Camera board driver swap (start_camera -> cam_rectify_stream), on the
#      camera SoC itself (192.168.127.10), since only cam_rectify_stream
#      exposes the TCP port this robot's camera receiver connects to.
#   2. Launch vision_node pointed at our own vision.yaml (via the
#      vision_config_path launch arg), since robocup_demo's default
#      vision.yaml has topic names (/StereoNetNode/...) that don't match
#      what the camera driver installed on this robot actually publishes
#      (/boostercamera/head/...). Our copy lives in build_no_ros/vision_config/
#      and is untouched by changes to the robocup_demo repo.
#   3. Launch vision_bridge_node, which subscribes to vision_node's
#      detections and writes them to /dev/shm/booster_vision_bridge.txt for
#      non-ROS2 processes (like visual_pass_ball_to_person) to read.
#   4. Launch vision_stream_server, a debug aid that overlays detection
#      boxes/labels/distance on the live camera feed and serves it as an
#      MJPEG-over-HTTP stream on port 8090 - view from a laptop via
#      `ssh -L 8090:localhost:8090 <user>@<robot-ip>` then open
#      http://localhost:8090 in a browser, no X11/GUI needed on either end.
#
# NOTE: the redundant host-side booster-x5-camera-depth.service (a duplicate
# of the camera receiver the vendor's own container stack already runs,
# which only caused connection contention) has been permanently disabled
# (systemctl disable --now), so this script no longer needs to stop it each
# run. If that's ever re-enabled for some reason, re-add a
# `sudo systemctl stop booster-x5-camera-depth.service` step here.
#
# The camera board driver swap (step 1) is still runtime-only
# (systemctl start/stop, not enable/disable), so it still needs to be
# redone after every camera board reboot.
#
# After this script finishes, run the actual behavior separately, in its own
# terminal:
#   cd ~/Workspace/build_no_ros && ./visual_pass_ball_to_person lo

set -e

CAMERA_BOARD_HOST="192.168.127.10"
CAMERA_BOARD_USER="root"
CAMERA_BOARD_PASS="br123456"

ROBOCUP_DEMO_DIR="$HOME/Workspace/booster_demo/robocup_demo"
BRIDGE_PKG_DIR="$HOME/Workspace/vision_ros_bridge_pkg"
VISION_CONFIG_DIR="$HOME/Workspace/build_no_ros/vision_config"

VISION_NODE_LOG="/tmp/pass_ball_to_person_vision_node.log"
VISION_BRIDGE_LOG="/tmp/pass_ball_to_person_vision_bridge.log"
VISION_STREAM_LOG="/tmp/pass_ball_to_person_vision_stream.log"

# `ros2 launch`/`ros2 run` don't exec into the actual node process - they
# spawn it as a *child*, so `kill` on just the top-level PID (what $! gives
# us below) leaves the real vision_node/vision_bridge_node binary running as
# an orphan. Pattern-match on the actual binary paths instead, which reaches
# the whole tree regardless of how it's nested. Run this before launching
# anything too, so re-running this script is always safe even if a previous
# run's cleanup was incomplete (this is what caused a stray old
# vision_bridge_node to keep racing a freshly-started one over the same
# bridge file).
cleanup_vision_pipeline() {
    # `|| true` on each line: with `set -e` active, pkill returning non-zero
    # (nothing matched - the common case) would otherwise abort the whole
    # script right here, before launching anything.
    pkill -9 -f "vision/lib/vision/vision_node" 2>/dev/null || true
    pkill -9 -f "ros2 launch vision launch.py" 2>/dev/null || true
    pkill -9 -f "vision_ros_bridge_pkg/lib/vision_ros_bridge_pkg/vision_bridge_node" 2>/dev/null || true
    pkill -9 -f "ros2 run vision_ros_bridge_pkg vision_bridge_node" 2>/dev/null || true
    pkill -9 -f "vision_ros_bridge_pkg/lib/vision_ros_bridge_pkg/vision_stream_server" 2>/dev/null || true
    pkill -9 -f "ros2 run vision_ros_bridge_pkg vision_stream_server" 2>/dev/null || true
}

echo "== [0/3] Cleaning up any leftover vision_node/vision_bridge_node from a previous run =="
cleanup_vision_pipeline
sleep 1

echo "== [1/3] Swapping camera board driver (start_camera -> cam_rectify_stream) =="
sshpass -p "$CAMERA_BOARD_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
    "${CAMERA_BOARD_USER}@${CAMERA_BOARD_HOST}" "
        systemctl stop start_camera.service
        systemctl start cam_rectify_stream.service
    "

echo "== [2/3] Launching vision_node with our own vision_config override =="
(
    cd "$ROBOCUP_DEMO_DIR"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    exec ros2 launch vision launch.py camera_type:=d-robotics show_det:=false save_depth:=false \
        vision_config_path:="$VISION_CONFIG_DIR"
) > "$VISION_NODE_LOG" 2>&1 &
VISION_NODE_PID=$!
echo "vision_node launch PID: $VISION_NODE_PID (log: $VISION_NODE_LOG)"

sleep 8

echo "== [3/4] Launching vision_bridge_node =="
(
    source /opt/ros/humble/setup.bash
    source "$ROBOCUP_DEMO_DIR/install/setup.bash"
    source "$BRIDGE_PKG_DIR/install/setup.bash"
    exec ros2 run vision_ros_bridge_pkg vision_bridge_node
) > "$VISION_BRIDGE_LOG" 2>&1 &
VISION_BRIDGE_PID=$!
echo "vision_bridge_node PID: $VISION_BRIDGE_PID (log: $VISION_BRIDGE_LOG)"

sleep 3

echo "== [4/4] Launching vision_stream_server (live annotated camera feed on :8090) =="
(
    source /opt/ros/humble/setup.bash
    source "$ROBOCUP_DEMO_DIR/install/setup.bash"
    source "$BRIDGE_PKG_DIR/install/setup.bash"
    exec ros2 run vision_ros_bridge_pkg vision_stream_server
) > "$VISION_STREAM_LOG" 2>&1 &
VISION_STREAM_PID=$!
echo "vision_stream_server PID: $VISION_STREAM_PID (log: $VISION_STREAM_LOG)"

sleep 2

echo ""
echo "Pipeline started. These run in the background; stop them with:"
echo "  pkill -9 -f vision/lib/vision/vision_node"
echo "  pkill -9 -f 'ros2 launch vision launch.py'"
echo "  pkill -9 -f vision_ros_bridge_pkg/lib/vision_ros_bridge_pkg/vision_bridge_node"
echo "  pkill -9 -f 'ros2 run vision_ros_bridge_pkg vision_bridge_node'"
echo "  pkill -9 -f vision_ros_bridge_pkg/lib/vision_ros_bridge_pkg/vision_stream_server"
echo "  pkill -9 -f 'ros2 run vision_ros_bridge_pkg vision_stream_server'"
echo "(plain 'kill $VISION_NODE_PID $VISION_BRIDGE_PID $VISION_STREAM_PID' is NOT"
echo " enough - those PIDs are just the ros2 launch/run wrapper, the actual node"
echo " binaries are separate child processes and would be left running as orphans)"
echo ""
echo "Watch for detections with:"
echo "  watch -n0.5 cat /dev/shm/booster_vision_bridge.txt"
echo ""
echo "View the live annotated camera feed from a laptop with:"
echo "  ssh -L 8090:localhost:8090 $(whoami)@$(hostname -I 2>/dev/null | awk '{print \$1}')"
echo "  then open http://localhost:8090 in a browser"
echo ""
echo "Then run the actual behavior in its own terminal:"
echo "  cd ~/Workspace/build_no_ros && ./visual_pass_ball_to_person lo"
