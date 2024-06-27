#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as n
import time
import sys
import math
import moveit_commander
import sensor_msgs.point_cloud2 as pc2
from copy import deepcopy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped


bridge = CvBridge()

desired_point = {'x': 0.5, 'y': 0.5, 'z': 0.1}


obj_detected = False
motion_computed = False

reference_frame = "world"
arm = moveit_commander.MoveGroupCommander("ur5_arm")
end_effector_link = arm.get_end_effector_link()

grasp = rospy.Publisher('grasp_commander', String, queue_size=1)


def Initialization():
    moveit_commander.roscpp_initialize(sys.argv)
    arm.set_pose_reference_frame(reference_frame)
    arm.allow_replanning(True)

    arm.set_goal_position_tolerance(0.05)
    arm.set_goal_orientation_tolerance(0.1)
    arm.set_planning_time(0.5)
    arm.set_max_acceleration_scaling_factor(0.1)
    arm.set_max_velocity_scaling_factor(0.1)
    print("Initialization...............................OK")

def set_pose_initial():
    default_joint_states = arm.get_current_joint_values()
    default_joint_states[0] = 1.5 #base link
    default_joint_states[1] = -1.5667 
    default_joint_states[2] = 0.89266 #can be used for take the camera up or down
    default_joint_states[3] = -0.87721
    default_joint_states[4] = -1.5705 #link after the gripper
    default_joint_states[5] = 0.0 #rotation of the gripper

    arm.set_joint_value_target(default_joint_states)

    arm.set_start_state_to_current_state()
    arm.go()
    print("Initial position...................................OK")
    grasp.publish('ok')
    time.sleep(7)


def object_detection(data):
    global xyz
    global obj_detected
    try:
        point_cloud = n.array(data)
        gen = pc2.read_points(data, skip_nans=True)
        int_data = list(gen)

        #points = [int(digit) for digit in str(point_cloud[0])]
        point_cloud = n.array_split(int_data, 3)
        
        points = point_cloud[0]
        point = points[0]

        x = point[2] - 0.35
        y = point[0]*(-0.1)
        z = (0.035895-0.205022)

        coordinate = [x, y, z]
        if coordinate == [5.800528526306152, 0.5329689502716065, 0.03589510200014]:
            print("any object detected")
            xyz = coordinate
            obj_detected = True

        else:
            print('object detected')
            print(coordinate)

        
        
    except Exception as e:
        print(e)

def compute_motion(xyz):
    global motion_computed
    
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]

    arm.set_pose_target([x, y, z, -math.pi/2, math.pi/2, 0.0])
    arm.go()
    time.sleep(15)
    motion_computed = True

def improved_motion(xyz):
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]

    pose_goal = Pose()
    pose_goal.orientation = arm.get_current_pose().pose.orientation
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm.set_pose_target(pose_goal)
    plan = arm.plan()

    success = arm.execute(plan[1], wait=True)
    arm.stop()
    arm.clear_pose_targets()

    while not success:  # FALLBACK FOR SAFETY
            arm.set_pose_target(pose_goal)
            plan = arm.plan()
            success = arm.execute(plan[1], wait=True)
            arm.stop()
            arm.clear_pose_targets()

    time.sleep(15)
    motion_computed = True

def image_callback(msg):
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    object_detection(cv2_img)

def Processus():
    if obj_detected == True:
        compute_motion(xyz)
        grasp.publish('you can take it!!!!!')
        time.sleep(5)
        set_pose_initial()
        obj_detected = False
        motion_computed = False
    else:
        pass


def main():
    rospy.init_node('image_listener')
    image_topic = "/camera_2_depth/color/image_raw"
    rospy.Subscriber(image_topic, PointCloud2, image_callback)

    global obj_detected
    global motion_computed
    global desired_point

    x = desired_point['x']
    y = desired_point['y']
    z = desired_point['z']

    xyz = [x, y, z]

    Initialization()
    set_pose_initial()
    compute_motion(xyz)
    grasp.publish('you can take it!!!!!')
    time.sleep(15)
    set_pose_initial()

    
    rospy.spin()

    

if __name__ == '__main__':
    main()
    rospy.spin()