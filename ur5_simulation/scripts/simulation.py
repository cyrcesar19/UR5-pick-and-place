#!/usr/bin/env python3

import sys
import moveit_commander
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Empty
from ur5_gripper_control.srv import FilterWorkspace, FilterWorkspaceRequest

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_place', anonymous=False)

# Octomap topics and services
camera_topics = ['camera_1_depth', 'camera_2_depth']
clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
publish_octomap = rospy.ServiceProxy('/filter_workspace', FilterWorkspace)

# Diagnostic publisher for waypoint poses
pose_pub = rospy.Publisher('/checker',PoseStamped,latch=True,queue_size=5)

# Initialise robot and move groups
robot = moveit_commander.robot.RobotCommander()
arm_group = moveit_commander.move_group.MoveGroupCommander("ur5_arm")
gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")



def command_gripper(state):
    value = [0.0]
    
    if state == 'open':
        value = [0.0]
        gripper_group.go(value, wait=True)
        gripper_group.stop()
        rospy.sleep(5)

    if state == 'close':
        value = [0.43]
        gripper_group.go(value, wait=True)
        gripper_group.stop()
        rospy.sleep(5)
    

def initial_pose():
    home_state = arm_group.get_current_state().joint_state
    home_state.name = list(home_state.name)[:6]
    home_state.position = [arm_group.get_named_target_values('home')['shoulder_pan_joint'],
                            arm_group.get_named_target_values('home')['shoulder_lift_joint'],
                            arm_group.get_named_target_values('home')['elbow_joint'],
                            arm_group.get_named_target_values('home')['wrist_1_joint'],
                            arm_group.get_named_target_values('home')['wrist_2_joint'],
                            arm_group.get_named_target_values('home')['wrist_3_joint']]
    plan = arm_group.plan(home_state)
    arm_group.execute(plan[1], wait=True)
    arm_group.stop()


def move_to_object(object):
    x = object[0]
    y = object[1]

    # Move to object
    pose_goal = Pose()
    pose_goal.orientation = arm_group.get_current_pose().pose.orientation
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = 0.34
    arm_group.set_pose_target(pose_goal)
    plan = arm_group.plan()
    arm_group.execute(plan[1], wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    
    # Close gripper
    command_gripper('close')       
    rospy.sleep(7)

    initial_pose()

    # Go to opposite side
    pose_goal.orientation = arm_group.get_current_pose().pose.orientation
    pose_goal.position.x = -x
    pose_goal.position.y = y
    pose_goal.position.z = 0.5
    arm_group.set_pose_target(pose_goal)
    plan = arm_group.plan()
    arm_group.execute(plan[1], wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    rospy.sleep(1)

    # Open gripper
    command_gripper('open')


if __name__ == "__main__":
    try:        
        print("Start...................")
        object1 = [0.5, -0.2]
        object2 = [0.7, 0.0]
        object3 = [0.5, 0.3]
            
        initial_pose()
        # Open gripper
        command_gripper('open')
        rospy.sleep(3)

        move_to_object(object1)
        initial_pose()
        move_to_object(object2)
        initial_pose()
        move_to_object(object3)
        initial_pose()


    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass