cmake_minimum_required(VERSION 3.8)
project(aeroterrabot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(aeroterrabot_interfaces REQUIRED)

# ── Hardware Interface Shared Library ─────────────────────────────────────────
add_library(aeroterrabot_hardware SHARED
  src/aeroterrabot_system_interface.cpp
)
target_include_directories(aeroterrabot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aeroterrabot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# ── Sensor Bridge Node ────────────────────────────────────────────────────────
add_executable(sensor_bridge_node src/sensor_bridge_node.cpp)
ament_target_dependencies(sensor_bridge_node
  rclcpp
  aeroterrabot_interfaces
)

# ── Plugin Export ─────────────────────────────────────────────────────────────
pluginlib_export_plugin_description_file(hardware_interface aeroterrabot_hardware.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS aeroterrabot_hardware
  EXPORT aeroterrabot_hardwareTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS sensor_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Ament Export ──────────────────────────────────────────────────────────────
ament_export_targets(aeroterrabot_hardwareTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
