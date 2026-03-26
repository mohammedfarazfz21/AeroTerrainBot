cmake_minimum_required(VERSION 3.8)
project(aeroterrabot_transform_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(aeroterrabot_interfaces REQUIRED)

# Transform Controller Node
add_executable(transform_controller src/transform_controller.cpp)
ament_target_dependencies(transform_controller
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  controller_manager_msgs
  aeroterrabot_interfaces
)

install(TARGETS
  transform_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
