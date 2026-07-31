cmake_minimum_required(VERSION 3.16)
project(visual_kick_no_ros CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 4.8 is installed at /usr/local (confirmed via
# `pkg-config --modversion opencv4`) but not on the default compiler
# search path, which is why __has_include(<opencv2/opencv.hpp>) in the
# .cpp silently fails and compiles out all the YOLO/camera code without
# find_package(OpenCV) here.
find_package(OpenCV REQUIRED)

add_executable(visual_kick_no_ros build_no_ros/visual_kick_no_ros.cpp)

# Adjust include/lib paths to your install
target_include_directories(visual_kick_no_ros PRIVATE /usr/local/include ${OpenCV_INCLUDE_DIRS})
target_link_directories(visual_kick_no_ros PRIVATE /usr/local/lib)

# libfastdds.* does not exist on this system - only libfastrtps.* does
# (confirmed via `find /usr/local/lib /usr/lib -iname "libfast*"`, and
# matches how test_shoot_sdk/booster_sdk_smoke_test already link).
target_link_libraries(visual_kick_no_ros
    booster_robotics_sdk
    fastrtps
    fastcdr
    pthread
    ${OpenCV_LIBS}
)

# Ball-to-person kick behavior: pure SDK now (B1LocoClient for locomotion/
# kick, ChannelFactory, raw FastDDS Kick publisher) - no ROS2 in this binary
# at all. Vision comes from a separate process, vision_bridge_node (built by
# ~/Workspace/vision_ros_bridge_pkg via colcon), which writes detections to
# /dev/shm/booster_vision_bridge.txt; this program just reads that file.
#
# ROS2 and the SDK cannot coexist in one process here: ROS2 Humble's
# rmw_fastrtps_cpp needs libfastrtps.so.2.6, while the SDK needs
# libfastrtps.so.2.13 - two ABI-incompatible versions that crash
# (std::bad_array_new_length, corrupted struct layout) the moment both
# create a DDS participant in the same process. See visual_pass_ball_to_
# person.cpp's vision section and vision_ros_bridge_pkg/src/vision_bridge_
# node.cpp for the full explanation. Splitting into two processes avoids
# that entirely - so no find_package(rclcpp), no ROS2 include/link paths,
# nothing ROS2-related anywhere in this file.
add_executable(visual_pass_ball_to_person build_no_ros/visual_pass_ball_to_person.cpp)

target_include_directories(visual_pass_ball_to_person PRIVATE /usr/local/include)
target_link_directories(visual_pass_ball_to_person PRIVATE /usr/local/lib)

target_link_libraries(visual_pass_ball_to_person
    booster_robotics_sdk
    fastrtps
    fastcdr
    pthread
)

# Minimal vision-only diagnostic (no locomotion, no kick) - isolates whether
# VisionClient::StartVisionService()/GetDetectionObject() work at all.
#
# Commented out: build_no_ros/vision_only_diagnostic.cpp no longer exists
# on disk (only its old compiled .o remains, from before it was removed) -
# this was blocking `cmake` from configuring at all, for every target in
# this file, not just this one. Not deleting the block in case the source
# turns up again - just disabling it until then.
# add_executable(vision_only_diagnostic build_no_ros/vision_only_diagnostic.cpp)
#
# target_include_directories(vision_only_diagnostic PRIVATE /usr/local/include)
# target_link_directories(vision_only_diagnostic PRIVATE /usr/local/lib)
#
# target_link_libraries(vision_only_diagnostic
#     booster_robotics_sdk
#     fastrtps
#     fastcdr
#     pthread
# )

add_executable(test build_no_ros/test.cpp)

target_include_directories(test PRIVATE /usr/local/include)
target_link_directories(test PRIVATE /usr/local/lib)

target_link_libraries(test
    booster_robotics_sdk
    fastrtps
    fastcdr
    pthread
)

# One-off diagnostic trigger: used to capture a real /LocoApiTopicReq
# message (via a separate ROS2 listener run alongside it) so a pure-ROS2
# equivalent can be built from a confirmed real request format instead of
# a guessed one. See build_no_ros/single_step.cpp's own header comment.
add_executable(single_step build_no_ros/single_step.cpp)

target_include_directories(single_step PRIVATE /usr/local/include)
target_link_directories(single_step PRIVATE /usr/local/lib)

target_link_libraries(single_step
    booster_robotics_sdk
    fastrtps
    fastcdr
    pthread
)