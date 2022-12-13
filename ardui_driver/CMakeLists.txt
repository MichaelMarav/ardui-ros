cmake_minimum_required(VERSION 3.0.2)
project(ardui_driver)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  ardui_msgs
)




catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ardui_driver
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs ardui_msgs 
)



include_directories(
 include  ${catkin_INCLUDE_DIRS} ardui_driver ardui_msgs
)

add_library(ardui_driver

  src/speed_controller.cpp
  src/driver.cpp
  src/remapper.cpp
  include/ardui_driver/remapper.h
  include/ardui_driver/speed_controller.h
  include/ardui_driver/driver.h
)

# PYTHON scripts
catkin_install_python(PROGRAMS scripts/initialize_servos.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# CPP excecutables 
add_executable(low_level_control src/nodes/low_level_control.cpp src/speed_controller.cpp src/remapper.cpp)
target_link_libraries(low_level_control ${ardui_driver_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(low_level_control ${catkin_EXPORTED_TARGETS} ardui_msgs_generate_messages_cpp)

add_executable(low_level_feedback  src/nodes/low_level_feedback.cpp src/driver.cpp src/remapper.cpp)
target_link_libraries(low_level_feedback ${ardui_driver_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(low_level_feedback ardui_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS} )


# add_executable(remapper src/remapper.cpp)
# target_link_libraries(remap_node ${catkin_LIBRARIES})
