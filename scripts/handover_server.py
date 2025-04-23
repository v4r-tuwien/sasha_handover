#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation

import math
import os
import sys

import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped
from hsrb_interface import Robot
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from tmc_manipulation_msgs.srv import SafeJointChange, SafeJointChangeRequest

from handover.msg import HandoverAction


class HandoverServer(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer('/handover', HandoverAction, self.execute, False)
        self.server.start()
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        self.force_thresh = 0.2
        self.position_reached = False
        self.finished = False

        #Robot initialization
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH
        self.gripper = self.robot.get('gripper')
        
        self.readjust_offset = rospy.ServiceProxy('/hsrb/wrist_wrench/readjust_offset', Empty)
        self.readjust_offset.wait_for_service(timeout = 5)
        self.joint_control = rospy.ServiceProxy('/safe_pose_changer/change_joint', SafeJointChange)
        self.joint_control.wait_for_service(timeout = 5)
        # Subscribe force torque sensor data from HSRB
        ft_sensor_topic = '/hsrb/wrist_wrench/compensated'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)
        
        if rospy.has_param('/handover/use_fancy_handover'):
            self.use_fancy_handover = rospy.get_param('/handover/use_fancy_handover')
        else:
            self.use_fancy_handover = True

        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def execute(self, goal):
        rospy.loginfo('Received a new goal.')

        if goal.force_thresh > 0:
            self.force_thresh = goal.force_thresh
        self.readjust_offset()
        self.move_to_handover_position(goal.object_name)
        while not self.finished:
            rospy.rostime.wallsleep(0.5)

        self.finished = False
        self.position_reached = False        
        self.server.set_succeeded()
        rospy.loginfo('Handover was successfully executed.')

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]
    
    def move_to_handover_position(self, object_name):

        if self.use_fancy_handover:
            self.whole_body.move_to_neutral()
            joint_goal = JointState()
            joint_goal.name.extend(['arm_flex_joint', 'arm_lift_joint', 'wrist_flex_joint', 
                                    'arm_roll_joint', 'wrist_roll_joint', 'head_pan_joint', 'head_tilt_joint'])
            joint_goal.position.extend([-0.3, 0.4, -1, 
                                        0, 0, 0, 0])
            req = SafeJointChangeRequest(joint_goal)
            res = self.joint_control(req) 
            self.position_reached = res.success
            
        if len(object_name) > 0:

            try:
                if "obj_" in object_name:
                    # Getting real object name from the params
                    dataset = rospy.get_param('/grasping_pipeline/dataset')
                    object_name = rospy.get_param(f'/object_mapping/{dataset}/{object_name}')
            except:
                rospy.loginfo("Object name not found in the params. Using the object name as it is.")

            if object_name[0].isdigit():
                object_name = object_name.split("_", 1)[-1]
            self.position_reached = True
            self.tts.say(f"You can take the {object_name} now.")
        else:
            self.tts.say('You can take the object now.')
        rospy.loginfo('You can take the object now.')
   

    def reset_offset(self):
        self.readjust_offset()

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z
        if (abs(self._force_data_x) > self.force_thresh or abs(self._force_data_y) > self.force_thresh or abs(self._force_data_z) > self.force_thresh) \
            and self.position_reached and not self.finished:
            self.gripper.command(1.0)
            self.finished = True
    
        

if __name__ == '__main__':
    rospy.init_node('handover_server')
    rospy.loginfo('Handover server started')
    handover = HandoverServer()
    rospy.spin()
