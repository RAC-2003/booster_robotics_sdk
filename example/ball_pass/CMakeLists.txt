cmake_minimum_required(VERSION 3.16)
project(visual_pass_ball_to_person CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# libfastdds.* does not exist on this system - only libfastrtps.* does
# (confirmed via `find /usr/local/lib /usr/lib -iname "libfast*"`).
target_link_libraries(visual_pass_ball_to_person
    booster_robotics_sdk
    fastrtps
    fastcdr
    pthread
)
