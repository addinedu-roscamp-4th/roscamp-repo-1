cmake_minimum_required(VERSION 3.8)
project(libro_picker_srvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MotorControl.srv"
  "srv/AngleControl.srv"
  "srv/PosControl.srv"
  "srv/GripperControl.srv"
  DEPENDENCIES std_msgs
)

ament_package()
