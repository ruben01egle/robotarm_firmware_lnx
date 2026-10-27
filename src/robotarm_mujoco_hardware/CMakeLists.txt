cmake_minimum_required(VERSION 3.8)
project(robotarm_mujoco_hardware)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mujoco_ros2_control REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/RobotarmMujocoSystem.cpp
  src/ServoBridge.cpp
  src/ServoPlugin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  mujoco_ros2_control::mujoco_ros2_control
  mujoco_ros2_control_plugins::mujoco_ros2_control_plugins
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  yaml-cpp::yaml-cpp
)

pluginlib_export_plugin_description_file(hardware_interface robotarm_mujoco_hardware_plugins.xml)
# the servo, run by mujoco_ros2_control on the physics thread (mujoco_plugins parameter)
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins servo_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_joint_servo test/test_joint_servo.cpp)
  target_link_libraries(test_joint_servo ${PROJECT_NAME})
endif()

ament_package()
