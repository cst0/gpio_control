cmake_minimum_required(VERSION 2.8.3)
project(gpio_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  InputState.msg
  OutputState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
