cmake_minimum_required(VERSION 3.8)
project(fr3_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(fr3_husky_msgs REQUIRED)

add_executable(ps4_input_manager src/ps4_input_manager.cpp)
ament_target_dependencies(ps4_input_manager
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  rclcpp_action
  franka_msgs
  fr3_husky_msgs
)

add_executable(gripper_manager src/gripper_manager.cpp)
ament_target_dependencies(gripper_manager
  rclcpp
  std_msgs
  rclcpp_action
  franka_msgs
)

install(TARGETS
  ps4_input_manager
  gripper_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()