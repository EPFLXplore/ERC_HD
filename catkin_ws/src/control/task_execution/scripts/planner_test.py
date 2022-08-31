#!/usr/bin/env python

import sys
import math
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from task_execution.srv import PoseGoal, PoseGoalResponse, JointGoal, JointGoalResponse
from task_execution.msg import Task
import task_execution.quaternion_arithmetic as qa
from task_execution.quaternion_arithmetic import normalize
import keyboard


pose_goal_pub = rospy.Publisher("/arm_control/pose_goal", geometry_msgs.msg.Pose, queue_size=5)
joint_goal_pub = rospy.Publisher("/arm_control/joint_goal", std_msgs.msg.Float64MultiArray, queue_size=5)
press_btn_pub = rospy.Publisher("/arm_control/task_assignment", Task, queue_size=5)
pos_manual_cmd_pub = rospy.Publisher("/arm_control/pos_manual_cmd", std_msgs.msg.Float32MultiArray, queue_size=1)
orient_manual_cmd_pub = rospy.Publisher("/arm_control/orient_manual_cmd", std_msgs.msg.Float32MultiArray, queue_size=1)


"""def quaternion(axis, angle):
    orientation = geometry_msgs.msg.Quaternion()
    axis = normalize(axis)
    orientation.w = math.cos(angle/2)
    orientation.x = axis[0]*math.sin(angle/2)
    orientation.y = axis[1]*math.sin(angle/2)
    orientation.z = axis[2]*math.sin(angle/2)
    return orientation

def normalize(axis):
    n = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    axis = (axis[0]/n, axis[1]/n, axis[2]/n)
    return axis"""

def req_joint_goal(angle):
    rospy.wait_for_service('/arm_control/joint_goal')
    try:
        proxy = rospy.ServiceProxy('/arm_control/joint_goal', JointGoal)
        cmd_id = 0
        goal = std_msgs.msg.Float64MultiArray()
        goal.data = [angle, 0, 0, 0, 0, 0]
        resp = proxy(cmd_id, goal)
        print(resp.ok)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def req_pose_goal(axis, angle):
    rospy.wait_for_service('/arm_control/pose_goal')
    try:
        proxy = rospy.ServiceProxy('/arm_control/pose_goal', PoseGoal)
        cmd_id = 0
        goal = geometry_msgs.msg.Pose()
        """goal.orientation.w = 1.0
        goal.position.x = 0.4
        goal.position.y = 0.1
        goal.position.z = 0.4
        if x == "x":
            goal.orientation.x = k
        elif x == "y":
            goal.orientation.y = k
        elif x == "z":
            goal.orientation.z = k
        elif x == "w":
            goal.orientation.w = k"""
        goal.orientation = quaternion(axis, angle)
        resp = proxy(cmd_id, goal, False, "aaaaaa")
        print(resp.ok)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def publish_joint_goal(angle):
    msg = std_msgs.msg.Float64MultiArray()
    msg.data = [angle,0,0,0,0,0]
    joint_goal_pub.publish(msg)

def publish_pose_goal():
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    pose_goal_pub.publish(pose_goal)

def publish_press_btn_task():
    task = Task()
    task.description = "btn"
    task.pose = geometry_msgs.msg.Pose()
    task.pose.position.x = 0.9
    task.pose.position.y = 0.1
    task.pose.position.z = 0.6
    task.pose.orientation = qa.quat([1, 1, 0], math.pi/3)
    press_btn_pub.publish(task)


def manual_inverse():
    pos_axis = [0, 0, 0]
    if keyboard.is_pressed("q"):
        pos_axis[0] = 1
    if keyboard.is_pressed("w"):
        pos_axis[1] = 1
    if keyboard.is_pressed("e"):
        pos_axis[2] = 1
    if keyboard.is_pressed("a"):
        pos_axis[0] = -1
    if keyboard.is_pressed("s"):
        pos_axis[1] = -1
    if keyboard.is_pressed("d"):
        pos_axis[2] = -1
    if sum(map(lambda x: x**2, pos_axis)):
        array = std_msgs.msg.Float32MultiArray()
        array.data = pos_axis + [1,]
        print("PUBLISHING")
        pos_manual_cmd_pub.publish(array)
    
    orient_axis = [0, 0, 0]
    if keyboard.is_pressed("7"):
        orient_axis[0] = 1
    if keyboard.is_pressed("8"):
        orient_axis[1] = 1
    if keyboard.is_pressed("9"):
        orient_axis[2] = 1
    if keyboard.is_pressed("4"):
        orient_axis[0] = -1
    if keyboard.is_pressed("5"):
        orient_axis[1] = -1
    if keyboard.is_pressed("6"):
        orient_axis[2] = -1
    if sum(map(lambda x: x**2, orient_axis)):
        array = std_msgs.msg.Float32MultiArray()
        array.data = orient_axis + [1,]
        print("PUBLISHING")
        #orient_manual_cmd_pub.publish(array)


def main():
    rospy.init_node("planner_test_node", anonymous=True)

    # publish_press_btn_task()
    """if sys.argv[1] == "j":
        req_joint_goal(int(sys.argv[2]))
    else:
        req_pose_goal((float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])), float(sys.argv[5]))"""
    
    rate = rospy.Rate(25)   # 25hz
    while not rospy.is_shutdown():
        manual_inverse()
        rate.sleep()


if __name__ == "__main__":
    main()
