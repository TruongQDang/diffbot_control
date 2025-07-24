cmake_minimum_required(VERSION 3.8)
project(diffbot_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorPos.msg"
  "msg/MotorVel.msg"
)

ament_package()
