cmake_minimum_required(VERSION 3.8)
project(path_follower_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(booster_msgs REQUIRED)
find_package(booster_interface REQUIRED)

# Step 5: publishes Move RPC requests on /LocoApiTopicReq (same mechanism
# single_step_pkg uses - this robot has no /cmd_vel), so booster_msgs is
# needed for RpcReqMsg. booster_interface is needed for RobotStatesMsg,
# used to confirm ChangeMode(kWalking) actually took effect by polling
# /robot_states (no publisher was ever found on /LocoApiTopicResp to wait
# on instead - checked live, Publisher count: 0).
add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower rclcpp nav_msgs booster_msgs booster_interface)

install(TARGETS path_follower
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()