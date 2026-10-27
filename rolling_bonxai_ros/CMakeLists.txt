cmake_minimum_required(VERSION 3.8)
project(rolling_bonxai_ros)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters features kdtree)
find_package(Eigen3 REQUIRED)

# Bonxai dependencies
find_package(bonxai_core REQUIRED)
find_package(bonxai_map REQUIRED)
find_package(rolling_bonxai REQUIRED)
find_package(rolling_bonxai_msgs REQUIRED)

# Create BonxaiServer library (for component and linking)
add_library(rolling_bonxai_icp src/icp_frontend.cpp src/sonar_clean_map.cpp)
set_target_properties(rolling_bonxai_icp PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(rolling_bonxai_icp PUBLIC cxx_std_17)
target_include_directories(rolling_bonxai_icp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(rolling_bonxai_icp PUBLIC ${PCL_LIBRARIES} bonxai_core::bonxai_core)

add_library(rolling_bonxai_server_lib SHARED
  src/rolling_bonxai_server.cpp
)

# Set C++17 standard
target_compile_features(rolling_bonxai_server_lib PUBLIC cxx_std_17)

# Include directories
target_include_directories(rolling_bonxai_server_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Link with Bonxai libraries
target_link_libraries(rolling_bonxai_server_lib
  PUBLIC
    rolling_bonxai::rolling_bonxai
    rolling_bonxai_icp
    ${PCL_LIBRARIES}
)

# Set ROS2 dependencies
ament_target_dependencies(rolling_bonxai_server_lib
  PUBLIC
    rclcpp
    rclcpp_components
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_msgs
    tf2_ros
    tf2_eigen
    pcl_conversions
    rolling_bonxai
    rolling_bonxai_msgs
)

# Register as composable node (for component containers)
rclcpp_components_register_node(rolling_bonxai_server_lib
  PLUGIN "RollingBonxai::RollingBonxaiServer"
  EXECUTABLE rolling_bonxai_server_component
)

# Create standalone executable from main.cpp
add_executable(rolling_bonxai_server_node
  src/main.cpp
)

# Link standalone executable to the library
target_link_libraries(rolling_bonxai_server_node
  rolling_bonxai_server_lib
)

# Install library
install(
  TARGETS rolling_bonxai_server_lib rolling_bonxai_icp
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install standalone executable
add_executable(compare_icp src/compare_icp.cpp)
target_link_libraries(compare_icp rolling_bonxai_icp rolling_bonxai::rolling_bonxai)

install(
  TARGETS rolling_bonxai_server_node compare_icp
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/prepare_icp_comparison.py scripts/report_icp_comparison.py
  scripts/validate_sonar_clean_reference.py scripts/report_sonar_clean.py
  scripts/prepare_sonar_playback.py
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install params files
install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Export targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
  PCL
  Eigen3
  bonxai_core
  bonxai_map
  rolling_bonxai
  rolling_bonxai_msgs
)

# Export include directories
ament_export_include_directories(include)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_body_tf_listener test/test_body_tf_listener.cpp)
  target_include_directories(test_body_tf_listener PRIVATE include)
  ament_target_dependencies(test_body_tf_listener rclcpp tf2 tf2_ros tf2_msgs)
  ament_add_gtest(test_icp_frontend test/test_icp_frontend.cpp)
  target_link_libraries(test_icp_frontend rolling_bonxai_icp rolling_bonxai::rolling_bonxai)
  ament_add_gtest(test_sonar_clean_map test/test_sonar_clean_map.cpp)
  target_link_libraries(test_sonar_clean_map rolling_bonxai_icp)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
