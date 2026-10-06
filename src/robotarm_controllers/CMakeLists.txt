cmake_minimum_required(VERSION 3.8)
project(robotarm_controllers)

# default to an optimized build, -DCMAKE_BUILD_TYPE=Debug etc. still overrides
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ruckig REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robotarm_interface REQUIRED)
find_package(robotarm_rbd REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/TeleopController.cpp
  src/CartesianJogController.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  realtime_tools
  rclcpp_lifecycle
  Eigen3
  robotarm_interface
  robotarm_rbd
)

target_link_libraries(${PROJECT_NAME}
  ruckig::ruckig
  Eigen3::Eigen
)

pluginlib_export_plugin_description_file(controller_interface robotarm_controllers_plugins.xml)

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

install(
  FILES robotarm_controllers_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  # Allocation check of CartesianJogController::update(). malloc is interposed in the test executable
  # (test/malloc_counter.cpp), so allocations inside the normally linked controller library, rclcpp,
  # ruckig and Eigen are counted too. Own main() (rclcpp::init), so gtest_main is not linked.
  ament_add_gtest(test_cartesian_jog_rt_alloc
    test/test_cartesian_jog_rt_alloc.cpp
    test/malloc_counter.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_include_directories(test_cartesian_jog_rt_alloc PRIVATE test)
  target_link_libraries(test_cartesian_jog_rt_alloc
    ${PROJECT_NAME}
    ament_index_cpp::ament_index_cpp
    Eigen3::Eigen
  )
  ament_target_dependencies(test_cartesian_jog_rt_alloc
    controller_interface
    hardware_interface
    lifecycle_msgs
    geometry_msgs
    rclcpp
  )
endif()

ament_package()
