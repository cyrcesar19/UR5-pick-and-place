#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import argparse
import time
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import control_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty

value = 0.0
obj_detected = False
group_name = "manipulator"
reference_frame = "world"
gripper = moveit_commander.MoveGroupCommander("gripper")


def grasp_object(msg):

    if type(msg) != float:
        test = str(msg)
        test = len(test)

        if test > 15:
            print ("grasping......................")
            value = 0.6

            close_gripper = [value]
            gripper.go(close_gripper, wait=True)
            gripper.stop()
            print("object grasped")

            time.sleep(7) 
            test = 0

        else:
            value = 0.0

            open_gripper = [value]
            gripper.go(open_gripper, wait=True)
            gripper.stop()
            print("object dropped")         


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('grasping',anonymous=True)
    grasping_state = "/grasp_commander"
    print("grasp subscribe....................")

    open_gripper = [value]
    gripper.go(open_gripper, wait=True)
    
    rospy.Subscriber(grasping_state, String, grasp_object)
    print("OK")

    rospy.spin()

    