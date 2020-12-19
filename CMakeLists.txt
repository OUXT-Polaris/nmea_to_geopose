cmake_minimum_required(VERSION 3.5)
project(nmea_to_geopose)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geodesy REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(nmea_to_geopose_component SHARED
  src/nmea_to_geopose_component.cpp)
target_compile_definitions(nmea_to_geopose_component
  PRIVATE "nmea_to_geopose_nmea_to_geopose_COMPONENT_BUILDING_DLL")
ament_target_dependencies(nmea_to_geopose_component
  rclcpp
  rclcpp_components
  nmea_msgs
  geodesy
  geographic_msgs
  quaternion_operation
  tf2_ros)
rclcpp_components_register_nodes(nmea_to_geopose_component
  "nmea_geopose::NmeaToGeoPoseComponent")

add_executable(nmea_to_geopose_node src/nmea_to_geopose_node.cpp)
ament_target_dependencies(nmea_to_geopose_node
  rclcpp
  rclcpp_components
  nmea_msgs
  geodesy
  geographic_msgs
  quaternion_operation
  tf2_ros)
target_link_libraries(nmea_to_geopose_node nmea_to_geopose_component glog)

install(TARGETS
  nmea_to_geopose_node
  DESTINATION lib/nmea_to_geopose
)
install(TARGETS
  nmea_to_geopose_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(nmea_to_geopose_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
