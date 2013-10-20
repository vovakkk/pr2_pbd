cmake_minimum_required(VERSION 2.8.3)
project(pr2_pbd_interaction)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
  std_msgs
  message_generation
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ActionStep.msg
  ActionStepSequence.msg
  ArmMode.msg
  ArmState.msg
  ArmTarget.msg
  ArmTrajectory.msg
  Condition.msg
  ExecutionStatus.msg
  ExperimentState.msg
  GripperAction.msg
  GripperState.msg
  GuiCommand.msg
  Object.msg
  RobotSound.msg
  Side.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetExperimentState.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check( launch/pbd_demo_desktop.launch )
roslaunch_add_file_check( launch/pbd_demo_robot.launch ROBOT=sim )
roslaunch_add_file_check( launch/pr2_interactive_manipulation_desktop.launch )
roslaunch_add_file_check( launch/simulated_robot.launch ROBOT=sim ROS_ENV_LOADER=/etc/ros/env.sh )
