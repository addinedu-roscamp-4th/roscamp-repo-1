cmake_minimum_required(VERSION 3.8)
project(libro_control_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Declare the list of messages to be built
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PinkyRequest.msg"
  "msg/PinkyResponse.msg"
  "msg/BookPickUp.msg"
  "msg/Navigate.msg"
  "msg/LibroRobotLog.msg"
  "msg/JetcobotPickRequest.msg"
  "msg/JetcobotPickResponse.msg"
  "msg/JetcobotPlaceRequest.msg"
  "msg/JetcobotPlaceResponse.msg"
  "msg/LibroRobotState.msg"

  DEPENDENCIES builtin_interfaces
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
