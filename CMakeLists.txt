cmake_minimum_required(VERSION 2.8.3)
project(handover)

find_package(catkin REQUIRED 
genmsg 
rospy
actionlib_msgs
geometry_msgs
grasping_pipeline_msgs
hsrb_interface_py
sensor_msgs)

catkin_package()
include_directories()

