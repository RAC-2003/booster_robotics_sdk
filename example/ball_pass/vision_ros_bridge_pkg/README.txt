# vision_ros_bridge_pkg

A small ROS 2 package that exists for one reason: to get data out of
`robocup_demo`'s ROS 2 vision pipeline and into a form that non-ROS2
executables (like `visual_pass_ball_to_person`, which links the Booster SDK)
can consume, without ever putting ROS 2 and the Booster SDK in the same
process.

## Why this package exists

ROS 2 Humble's `rmw_fastrtps_cpp` is built against `libfastrtps.so.2.6`. The
Booster SDK (`ChannelFactory`/`B1LocoClient`) links `libfastrtps.so.2.13` -
a different, ABI-incompatible version of the same library. Loading both in
one process crashes (`std::bad_array_new_length`, deep inside
`rmw_fastrtps_shared_cpp::create_participant`, from corrupted struct layout).

So every executable in this package is a **standalone process** that only
ever touches ROS 2 topics. Each one writes its output somewhere a completely
separate, SDK-linked process can read it from (a file in `/dev/shm`, an HTTP
stream, or a directory of files on disk) - never a shared library, never a
shared process.

## The four executables

### `vision_bridge_node`
Subscribes to `robocup_demo`'s detection topic (`/booster_soccer/detection`)
and writes the latest detections to `/dev/shm/booster_vision_bridge.txt` on
every message.

- **File format** (plain text, no JSON dependency needed on the reading
  side):
  ```
  <write_ts_ms> <capture_ts_ms>
  <label>|<confidence>|<x>,<y>,<z>
  <label>|<confidence>|<x>,<y>,<z>
  ...
  ```
  `write_ts_ms` is when this process wrote the file (for a staleness check);
  `capture_ts_ms` is the original camera frame's own timestamp, so a reader
  can measure true end-to-end latency, not just the last-leg gap. Position
  falls back to `position_projection` when `position` is all-zero (i.e.
  depth-based estimation is off, `use_depth: false`).
- Written to a `.tmp` file and `rename()`d over the real path, so a reader
  never sees a half-written file.
- **Optional pose/calibration recording**, off by default: set the
  `pose_output_basename` ROS 2 parameter to a non-empty path prefix and it
  also subscribes to `/booster_soccer/t_head2base` and
  `/booster_soccer/cal_param`, appending a CSV row per message to
  `<basename>_pose.csv` and `<basename>_calparam.csv` (two independent
  files, not merged/synced).

### `vision_stream_server`
Subscribes to the camera image topic *and* the detection topic, draws
bounding boxes + label + confidence + distance on each frame, and serves the
annotated video as an MJPEG-over-HTTP stream on **port 8090** - viewable
from any browser, including remotely over an SSH port forward
(`ssh -L 8090:localhost:8090 <user>@<robot-ip>`, then open
`http://localhost:8090`), no X11/GUI needed on either end.

- `color_topic` (ROS 2 parameter, default `/boostercamera/head/rgb`).
- Detections older than 1 second aren't drawn (a stale box on a live frame
  is more misleading than a frame with no box at all).
- Decodes `nv12` frames itself (`cv_bridge` has no built-in NV12 decoder).

### `raw_stream_server`
Same idea as `vision_stream_server`, but subscribes to the image topic
**only** - no detections, no overlay, unmodified pixels - for capturing
ground-truth footage where the exact camera output matters (e.g. a running
track's lane lines). Serves on **port 8091**, at maximum JPEG quality, and
pushes every frame as it arrives (no fps cap), since this is meant to be
recorded frame-accurately rather than watched live. Both `color_topic` and
`http_port` are ROS 2 parameters, so a second instance can run alongside the
first on a different port (e.g. pointed at the right-eye topic for stereo
recording).

### `depth_recorder`
Subscribes to the camera driver's stereo-computed depth topic
(`/boostercamera/head/depth`, default) and writes each frame's raw bytes to
disk, losslessly - no video encoding, since that would corrupt real
per-pixel distance values. This depth topic comes from the camera driver
itself (`cam_stream_receiver_ros2`), not from `vision_node`, and carries
real data independent of `vision_node`'s own `use_depth` config flag.

- Usage: `./depth_recorder <output_dir>` (created if it doesn't exist).
- Writes `<output_dir>/info.txt` once (width/height/encoding/step from the
  first frame), then one `<output_dir>/<epoch_ms>.raw` per frame after that
  - exactly `msg->data`, no interpretation. A reader (e.g. numpy) uses
    `info.txt`'s `encoding` field (e.g. `16UC1`/`32FC1`) to know how to
    interpret the raw bytes.
- `depth_topic` is a ROS 2 parameter if it's ever published somewhere else.

## How it fits together

```
robocup_demo (vision_node, ROS 2)
        |
        |  /booster_soccer/detection, /boostercamera/head/rgb, .../depth
        v
 vision_ros_bridge_pkg  (this package - ROS 2 process(es))
        |
        |-- vision_bridge_node   -> /dev/shm/booster_vision_bridge.txt
        |-- vision_stream_server -> http://<robot>:8090/  (annotated MJPEG)
        |-- raw_stream_server    -> http://<robot>:8091/  (raw MJPEG)
        |-- depth_recorder       -> <output_dir>/*.raw + info.txt
        v
visual_pass_ball_to_person  (SDK-linked, non-ROS2 process)
  reads /dev/shm/booster_vision_bridge.txt directly - no ROS 2 in this
  process, no FastDDS version conflict.
```

Only `vision_bridge_node` feeds the actual ball-passing behavior
(`visual_pass_ball_to_person`) - the two stream servers and the depth
recorder are debugging/data-capture tools, not part of the control loop.

## Building

Standard `ament_cmake` package:
```bash
colcon build --packages-select vision_ros_bridge_pkg
```
Executables install to `lib/vision_ros_bridge_pkg/` (`vision_bridge_node`,
`vision_stream_server`, `raw_stream_server`, `depth_recorder`).

## Quick reference

| Executable | Output | Port | Depends on |
|---|---|---|---|
| `vision_bridge_node` | `/dev/shm/booster_vision_bridge.txt` (+ optional CSVs) | - | `vision_interface`, `geometry_msgs` |
| `vision_stream_server` | MJPEG-over-HTTP, annotated | 8090 | `vision_interface`, `cv_bridge`, `OpenCV`, `sensor_msgs` |
| `raw_stream_server` | MJPEG-over-HTTP, unmodified | 8091 | `cv_bridge`, `OpenCV`, `sensor_msgs` |
| `depth_recorder` | `<output_dir>/*.raw` + `info.txt` | - | `sensor_msgs` |
