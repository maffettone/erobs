cmake_minimum_required(VERSION 3.8)
project(pdf_beamtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hello_moveit_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  moveit_ros_planning_interface
  hello_moveit_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  yaml-cpp
)

add_executable(pick_place_action_server src/pick_place_server.cpp)

ament_target_dependencies(pick_place_action_server ${dependencies})
target_link_libraries(pick_place_action_server yaml-cpp)

add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(hello_moveit ${dependencies})


install(TARGETS
  hello_moveit
  pick_place_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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
