cmake_minimum_required(VERSION 3.8)
project(system_data_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)

include_directories(include)

add_library(sdr_component SHARED src/sdr_component.cpp)
target_compile_definitions(sdr_component PRIVATE "SDR_BUILDING_DLL")
ament_target_dependencies(sdr_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rosbag2_cpp
  rosbag2_transport
)
rclcpp_components_register_nodes(sdr_component "sdr::SystemDataRecorder")

add_executable(system_data_recorder src/system_data_recorder.cpp)
target_link_libraries(system_data_recorder sdr_component)
ament_target_dependencies(system_data_recorder
  rclcpp
)

install(TARGETS
  sdr_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  system_data_recorder
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
