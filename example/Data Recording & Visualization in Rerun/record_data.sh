#!/usr/bin/env bash
#
# record_k1_stream_reliable.sh
#
# Camera-first ROS 2 recorder for Booster K1.
# Designed to improve reliability for image/video streaming topics.
#
# Usage:
#   ./record_k1_stream_reliable.sh FIELD_002
#
# Optional environment variables:
#   K1_DATA_ROOT=/path/to/data
#   STREAM_STATUS_INTERVAL_SEC=10
#   STREAM_MIN_FREE_DISK_GB=15
#   STREAM_MIN_AVAILABLE_RAM_MB=1200
#
# Notes:
# - This script intentionally records a smaller topic set.
# - Records the combined/packed stereo frame (/boostercamera/head/combine/rgb)
#   as video - not the compressed /booster_video_stream relay (confirmed
#   via live node introspection to just be a compressed copy of the left
#   lens alone), and not the plain right-lens topic
#   (/boostercamera/head/right/rgb, confirmed dead - has a registered
#   publisher, StereoNetNode, that never actually sends any frames).
#   NOTE: this combined topic's exact internal pixel layout (e.g.
#   side-by-side vs something else) has not been verified - check before
#   assuming it can be processed like a normal single-camera image.
# - This is a raw sensor_msgs/Image topic, the same kind that was
#   confirmed earlier to be prone to dropped frames under load (unlike
#   the compressed stream) - if recordings come out choppy/sparse, that
#   tradeoff is why.
# - Requires the camera board to be in its rectify-streaming mode for
#   this topic to carry any real data at all (confirmed: without this
#   switch, the raw head camera topics recorded zero messages in an
#   earlier test) - this script does that switch itself, same fix already
#   applied to record_k1_field_onboard.sh.

set -euo pipefail

RUN_ID="${1:-FIELD_$(date +%Y%m%d_%H%M%S)}"

COMBINE_CAMERA_TOPIC="/boostercamera/head/combine/rgb"

DATA_ROOT="${K1_DATA_ROOT:-$HOME/k1_field_data}"
RUN_DIR="${DATA_ROOT}/${RUN_ID}"
BAG_DIR="${RUN_DIR}/rosbag"
LOG_FILE="${RUN_DIR}/recorder.log"
METADATA_FILE="${RUN_DIR}/metadata.txt"
QOS_FILE="${RUN_DIR}/qos_overrides.yaml"

# Conservative safety limits for long field sessions.
MIN_FREE_DISK_GB="${STREAM_MIN_FREE_DISK_GB:-15}"
MIN_AVAILABLE_RAM_MB="${STREAM_MIN_AVAILABLE_RAM_MB:-1200}"
STATUS_INTERVAL_SEC="${STREAM_STATUS_INTERVAL_SEC:-10}"

# Larger cache helps with short write bursts from video streams.
MAX_CACHE_BYTES=1073741824         # 1 GB
SPLIT_BAG_BYTES=2147483648         # 2 GB
MAX_DURATION_SEC=0                 # 0 = until Ctrl+C or safety stop

# MCAP config tuned for recording reliability (lower CPU pressure).
MCAP_STREAM_CONFIG="${HOME}/k1_tools/mcap_streaming_safe.yaml"

# Camera board (the head camera's own onboard computer, separate from K1's
# main computer this script runs on) - same driver switch as
# record_k1_field_onboard.sh, needed before /boostercamera/head/combine/rgb
# has any real frames flowing.
CAMERA_BOARD_HOST="192.168.127.10"
CAMERA_BOARD_USER="root"
CAMERA_BOARD_PASS="br123456"

BAG_PID=""
STOP_REASON="Normal user stop or recorder exit"

# Core telemetry kept intentionally small.
BASE_TOPICS=(
  "/imu/data"
  "/odom"
  "/joint_states"
  "/tf"
  "/tf_static"
)

require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "ERROR: Required command '$1' was not found." >&2
    exit 1
  }
}

log_message() {
  local MESSAGE="$1"
  echo "$(date '+%Y-%m-%d %H:%M:%S') | ${MESSAGE}" | tee -a "${LOG_FILE}"
}

get_free_disk_gb() {
  df -BG "${DATA_ROOT}" | awk 'NR==2 {gsub(/G/, "", $4); print $4}'
}

get_available_ram_mb() {
  awk '/MemAvailable:/ {printf "%.0f", $2/1024}' /proc/meminfo
}

get_bag_size_mb() {
  if [ -d "${BAG_DIR}" ]; then
    du -sm "${BAG_DIR}" 2>/dev/null | awk '{print $1}'
  else
    echo "0"
  fi
}

stop_recorder() {
  if [ -n "${BAG_PID}" ] && kill -0 "${BAG_PID}" 2>/dev/null; then
    log_message "Stopping rosbag recorder cleanly: ${STOP_REASON}"
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
}

on_interrupt() {
  STOP_REASON="Stopped by user using Ctrl+C"
  stop_recorder
  exit 0
}

on_exit() {
  stop_recorder
}

mkdir -p "${RUN_DIR}"
touch "${LOG_FILE}"

trap on_interrupt INT TERM
trap on_exit EXIT

require_command ros2
require_command df
require_command awk
require_command du
require_command grep
require_command tee
require_command sshpass

ros2 daemon start >/dev/null 2>&1 || true

FREE_DISK_GB="$(get_free_disk_gb)"
if [ -z "${FREE_DISK_GB}" ] || [ "${FREE_DISK_GB}" -lt "${MIN_FREE_DISK_GB}" ]; then
  log_message "ERROR: Free disk is ${FREE_DISK_GB:-unknown} GB, minimum is ${MIN_FREE_DISK_GB} GB."
  exit 1
fi

# Not fatal if this fails - the rest of the topics (joint states, IMU,
# etc.) are still worth recording even without camera data, so this logs
# a warning and continues rather than aborting the run.
log_message "Switching camera board driver (start_camera -> cam_rectify_stream)."
if sshpass -p "${CAMERA_BOARD_PASS}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
    "${CAMERA_BOARD_USER}@${CAMERA_BOARD_HOST}" "
        systemctl stop start_camera.service
        systemctl start cam_rectify_stream.service
    " >> "${LOG_FILE}" 2>&1; then
  log_message "Camera board driver switched successfully."
else
  log_message "WARNING: Could not switch camera board driver - camera topics may record no data."
fi

AVAILABLE_TOPICS="$(ros2 topic list || true)"
if ! grep -Fxq "${COMBINE_CAMERA_TOPIC}" <<< "${AVAILABLE_TOPICS}"; then
  log_message "ERROR: Combine camera topic '${COMBINE_CAMERA_TOPIC}' is not visible."
  log_message "Run: ros2 topic list -t"
  exit 1
fi

TOPICS_TO_RECORD=("${COMBINE_CAMERA_TOPIC}")
for TOPIC in "${BASE_TOPICS[@]}"; do
  if grep -Fxq "${TOPIC}" <<< "${AVAILABLE_TOPICS}"; then
    TOPICS_TO_RECORD+=("${TOPIC}")
  fi
done

cat > "${QOS_FILE}" <<EOF
${COMBINE_CAMERA_TOPIC}:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile
EOF

# Create a low-CPU MCAP config if it does not exist.
if [ ! -f "${MCAP_STREAM_CONFIG}" ]; then
  mkdir -p "$(dirname "${MCAP_STREAM_CONFIG}")"
  cat > "${MCAP_STREAM_CONFIG}" <<EOF
compression: "None"
chunkSize: 16777216
noChunkCRC: true
enableDataCRC: false
noSummaryCRC: true
EOF
fi

cat > "${METADATA_FILE}" <<EOF
run_id: ${RUN_ID}
recorded_on: Booster K1 onboard computer
date_local: $(date +"%Y-%m-%d %H:%M:%S %Z")
date_utc: $(date -u +"%Y-%m-%dT%H:%M:%SZ")
hostname: $(hostname)
ros_domain_id: ${ROS_DOMAIN_ID:-0}
data_root: ${DATA_ROOT}
mode: streaming_reliable
combine_camera_topic: ${COMBINE_CAMERA_TOPIC}
qos_overrides_path: ${QOS_FILE}
max_cache_bytes: ${MAX_CACHE_BYTES}
split_bag_bytes: ${SPLIT_BAG_BYTES}

recorded_topics:
$(printf '  - %s\n' "${TOPICS_TO_RECORD[@]}")
EOF

echo
echo "============================================================"
echo "BOOSTER K1 STREAM-RELIABLE RECORDER"
echo "============================================================"
echo "Run ID:          ${RUN_ID}"
echo "Run directory:   ${RUN_DIR}"
echo "Combine camera:  ${COMBINE_CAMERA_TOPIC}"
echo "Recorded topics: ${#TOPICS_TO_RECORD[@]}"
echo "Cache:           $((MAX_CACHE_BYTES / 1024 / 1024)) MB"
echo "Split size:      $((SPLIT_BAG_BYTES / 1024 / 1024 / 1024)) GB"
echo "============================================================"
echo

printf "Selected topics:\n"
printf "  %s\n" "${TOPICS_TO_RECORD[@]}"
echo

echo "Quick source-rate check (5 seconds each):"
if command -v timeout >/dev/null 2>&1; then
  timeout 5 ros2 topic hz "${COMBINE_CAMERA_TOPIC}" || true
else
  echo "timeout command not available; skipping quick hz probe."
fi
echo

read -rp "Press ENTER to start recording, or Ctrl+C to cancel... "

log_message "Starting streaming-reliable recording."
log_message "Combine camera topic: ${COMBINE_CAMERA_TOPIC}"
log_message "Selected topics: ${TOPICS_TO_RECORD[*]}"

if ros2 bag record -s mcap --help >/dev/null 2>&1; then
  log_message "Storage: MCAP with ${MCAP_STREAM_CONFIG}"

  if [ "${MAX_DURATION_SEC}" -gt 0 ]; then
    ros2 bag record \
      -s mcap \
      --storage-config-file "${MCAP_STREAM_CONFIG}" \
      --qos-profile-overrides-path "${QOS_FILE}" \
      --max-cache-size "${MAX_CACHE_BYTES}" \
      --max-bag-size "${SPLIT_BAG_BYTES}" \
      --max-bag-duration "${MAX_DURATION_SEC}" \
      -o "${BAG_DIR}" \
      "${TOPICS_TO_RECORD[@]}" \
      >> "${LOG_FILE}" 2>&1 &
  else
    ros2 bag record \
      -s mcap \
      --storage-config-file "${MCAP_STREAM_CONFIG}" \
      --qos-profile-overrides-path "${QOS_FILE}" \
      --max-cache-size "${MAX_CACHE_BYTES}" \
      --max-bag-size "${SPLIT_BAG_BYTES}" \
      -o "${BAG_DIR}" \
      "${TOPICS_TO_RECORD[@]}" \
      >> "${LOG_FILE}" 2>&1 &
  fi
else
  log_message "WARNING: MCAP plugin unavailable; using default rosbag storage."

  if [ "${MAX_DURATION_SEC}" -gt 0 ]; then
    ros2 bag record \
      --qos-profile-overrides-path "${QOS_FILE}" \
      --max-cache-size "${MAX_CACHE_BYTES}" \
      --max-bag-size "${SPLIT_BAG_BYTES}" \
      --max-bag-duration "${MAX_DURATION_SEC}" \
      -o "${BAG_DIR}" \
      "${TOPICS_TO_RECORD[@]}" \
      >> "${LOG_FILE}" 2>&1 &
  else
    ros2 bag record \
      --qos-profile-overrides-path "${QOS_FILE}" \
      --max-cache-size "${MAX_CACHE_BYTES}" \
      --max-bag-size "${SPLIT_BAG_BYTES}" \
      -o "${BAG_DIR}" \
      "${TOPICS_TO_RECORD[@]}" \
      >> "${LOG_FILE}" 2>&1 &
  fi
fi

BAG_PID=$!
log_message "rosbag PID: ${BAG_PID}"

echo
echo "Recording is active. Press Ctrl+C to stop safely."
echo "Live status interval: ${STATUS_INTERVAL_SEC} seconds."
echo

while kill -0 "${BAG_PID}" 2>/dev/null; do
  FREE_DISK_GB="$(get_free_disk_gb)"
  AVAILABLE_RAM_MB="$(get_available_ram_mb)"
  BAG_SIZE_MB="$(get_bag_size_mb)"

  log_message "Bag: ${BAG_SIZE_MB} MB | Free disk: ${FREE_DISK_GB} GB | Available RAM: ${AVAILABLE_RAM_MB} MB"

  if [ "${FREE_DISK_GB}" -lt "${MIN_FREE_DISK_GB}" ]; then
    STOP_REASON="Critical disk threshold reached: ${FREE_DISK_GB} GB free"
    stop_recorder
    break
  fi

  if [ "${AVAILABLE_RAM_MB}" -lt "${MIN_AVAILABLE_RAM_MB}" ]; then
    STOP_REASON="Critical RAM threshold reached: ${AVAILABLE_RAM_MB} MB available"
    stop_recorder
    break
  fi

  sleep "${STATUS_INTERVAL_SEC}"
done

wait "${BAG_PID}" 2>/dev/null || true
BAG_PID=""

FREE_DISK_AFTER_GB="$(get_free_disk_gb)"
BAG_SIZE_FINAL_MB="$(get_bag_size_mb)"

log_message "Recording completed."
log_message "Final bag size: ${BAG_SIZE_FINAL_MB} MB"
log_message "Free disk after recording: ${FREE_DISK_AFTER_GB} GB"

echo
echo "============================================================"
echo "RECORDING COMPLETE"
echo "============================================================"
echo "Bag directory: ${BAG_DIR}"
echo "Metadata:      ${METADATA_FILE}"
echo "Recorder log:  ${LOG_FILE}"
echo "QoS file:      ${QOS_FILE}"
echo
echo "Verify message counts by topic:"
echo "  ros2 bag info ${BAG_DIR}"
echo
