cmake_minimum_required(VERSION 3.0.2)
project(ardui_msgs)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  AnalogFeedback.msg
  GoalStates.msg
  JointStates.msg
  ServoCommand.msg
) 

generate_messages(
  DEPENDENCIES
  # Add here the dependecies that are inside the message
  # e.g. if you need geometry_msgs inside the custom .msg add geometry msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


