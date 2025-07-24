cmake_minimum_required(VERSION 3.8)
project(diffbot_control)

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  hardware_interface
  diffbot_interfaces
  ament_cmake
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Compile
add_library(
  diffbot_hardware_interface SHARED
  hardware_interface/diffbot_hardware_interface.cpp
)
target_compile_features(diffbot_hardware_interface PUBLIC cxx_std_17)
target_include_directories(diffbot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware_interface/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(diffbot_hardware_interface PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${diffbot_interfaces_TARGETS}
)

# Install
install(
  DIRECTORY hardware_interface/include
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config bringup/rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY description/ros2_control description/urdf description/world
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS diffbot_hardware_interface
  EXPORT export_diffbot_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export
pluginlib_export_plugin_description_file(hardware_interface diffbot_hardware_interface.xml)
ament_export_targets(export_diffbot_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
