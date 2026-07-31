cmake_minimum_required(VERSION 3.8)
project(vision_ros_bridge_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_interface REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Standalone process: subscribes to /booster_soccer/detection and writes the
# latest detections to a tmpfs file. Runs as its own process rather than
# being linked into visual_pass_ball_to_person, because ROS2 Humble's
# rmw_fastrtps_cpp needs libfastrtps.so.2.6 while the Booster SDK needs
# libfastrtps.so.2.13 - two ABI-incompatible versions that crash
# (std::bad_array_new_length, corrupted struct layout) if both are ever
# loaded and creating DDS participants in the same process. See
# vision_bridge_node.cpp for the full explanation.
#
# geometry_msgs added for its optional pose-recording feature
# (/booster_soccer/t_head2base, TransformStamped) - vision_interface
# already covers /booster_soccer/cal_param (CalParam).
add_executable(vision_bridge_node src/vision_bridge_node.cpp)
ament_target_dependencies(vision_bridge_node rclcpp vision_interface geometry_msgs)

# Debug/visualization aid: annotates the live camera feed with detection
# boxes + distance and serves it as an MJPEG-over-HTTP stream, viewable from
# any browser (including remotely over an SSH port forward). Same
# out-of-process requirement as vision_bridge_node above.
add_executable(vision_stream_server src/vision_stream_server.cpp)
ament_target_dependencies(vision_stream_server rclcpp vision_interface cv_bridge sensor_msgs OpenCV)
target_link_libraries(vision_stream_server pthread)

# Recording source: same image topic as vision_stream_server, but no
# detections subscription and no overlay drawing - serves the camera feed
# exactly as received, for capturing ground-truth footage (e.g. a running
# track's lane lines) rather than a live debug view. Own port (8091) so it
# can coexist with vision_stream_server (8090).
add_executable(raw_stream_server src/raw_stream_server.cpp)
ament_target_dependencies(raw_stream_server rclcpp cv_bridge sensor_msgs OpenCV)
target_link_libraries(raw_stream_server pthread)

# Recording source for the camera driver's stereo-computed depth topic
# (/boostercamera/head/depth) - writes raw per-frame binary, not a video
# encoding, since depth values need to stay lossless. No cv_bridge/OpenCV
# needed (unlike the color streamers above) since this never decodes the
# image, just writes msg->data verbatim.
add_executable(depth_recorder src/depth_recorder.cpp)
ament_target_dependencies(depth_recorder rclcpp sensor_msgs)

install(TARGETS vision_bridge_node vision_stream_server raw_stream_server depth_recorder
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
