cmake_minimum_required(VERSION 3.0.2)
project(ur5_bringup)


find_package(catkin REQUIRED 
  roscpp
  configuration_msgs
  moveit_planning_helper
  )


catkin_package()

include_directories(
${catkin_INCLUDE_DIRS}
)
add_executable(control_switch src/control_switch.cpp)
target_link_libraries(control_switch ${catkin_LIBRARIES})

