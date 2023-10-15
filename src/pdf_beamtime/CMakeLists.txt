cmake_minimum_required(VERSION 3.8)
project(pdf_beamtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pdf_beamtime_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  moveit_ros_planning_interface
  pdf_beamtime_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  yaml-cpp
)

include_directories(include)  # This makes sure the libraries are visible

add_executable(pdf_beamtime_server src/pdf_beamtime_server.cpp)
ament_target_dependencies(pdf_beamtime_server ${dependencies})

add_executable(pdf_beam_env src/pdf_beam_env.cpp)
ament_target_dependencies(pdf_beam_env ${dependencies})

install(TARGETS
  pdf_beamtime_server
  pdf_beam_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
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
