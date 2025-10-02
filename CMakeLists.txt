cmake_minimum_required(VERSION 3.8)
project(spot_multi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration)
find_package(small_gicp REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_processor src/point_cloud_processor.cpp)
ament_target_dependencies(point_cloud_processor
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
  message_filters
  pcl_conversions
  pcl_ros
)
target_link_libraries(point_cloud_processor
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
)

install(TARGETS
  point_cloud_processor
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

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/spot_sync_drive.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/spot_graphnav_loc.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
