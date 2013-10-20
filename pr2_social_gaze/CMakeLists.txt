cmake_minimum_required(VERSION 2.8.3)
project(pr2_social_gaze)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  actionlib
  actionlib_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Gaze.action
)

generate_messages(
  DEPENDENCIES 
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
)

