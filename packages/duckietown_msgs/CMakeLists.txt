cmake_minimum_required(VERSION 3.5)
project(duckietown_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelEncoderStamped.msg"
  "msg/WheelsCmdStamped.msg"
  DEPENDENCIES std_msgs
)

ament_package()
