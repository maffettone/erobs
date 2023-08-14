cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

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


set(dependencies
  moveit_ros_planning_interface
  hello_moveit_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
)


add_library(${PROJECT_NAME}_pick_place_repeat_action_server SHARED src/pick_place_repeat_server.cpp)
target_include_directories(${PROJECT_NAME}_pick_place_repeat_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME}_pick_place_repeat_action_server PRIVATE "HELLO_MOVEIT_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME}_pick_place_repeat_action_server ${dependencies})
rclcpp_components_register_nodes(${PROJECT_NAME}_pick_place_repeat_action_server "hello_moveit::PickPlaceRepeatActionServer" EXECUTABLE pick_place_repeat_action_server)


add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(hello_moveit ${dependencies})

add_executable(pose_subscriber src/pose_subscriber.cpp)
ament_target_dependencies( pose_subscriber ${dependencies})

add_executable(parameter_demo src/parameter_demo.cpp)
ament_target_dependencies(parameter_demo ${dependencies})

install(TARGETS 
  hello_moveit
  parameter_demo
  pose_subscriber
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_pick_place_repeat_action_server 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
