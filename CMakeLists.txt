cmake_minimum_required(VERSION 3.8)
project(depth-filter-scan-converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(depth_to_scan_node src/depth_to_scan.cpp)
ament_target_dependencies(depth_to_scan_node 
  rclcpp sensor_msgs 
  rosgraph_msgs 
  pcl_conversions 
  PCL   
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(depth_to_scan_node
  ${Boost_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

# Install directories
install(
  DIRECTORY src launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install executable
install(TARGETS
  depth_to_scan_node
  DESTINATION lib/${PROJECT_NAME}
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
endif()

ament_package()
