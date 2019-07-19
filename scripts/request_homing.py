#!/usr/bin/env python
#-*- encoding: utf8 -*-

import sys
import rospy
import actionlib

from dynamixel_ros_control.msg import HomingAction, HomingActionGoal, HomingActionFeedback

def func_done(state, result):
	rospy.loginfo('done')

def func_feedback(feedback):
	rospy.loginfo('feedback')

def func_active():
	rospy.loginfo("start")


def main():
    if(len(sys.argv) == 1 or len(sys.argv) > 2):
        rospy.logwarn('please specify motor name. ex) request_homing.py motor_name')
        exit(1)

    target_motor = str(sys.argv[1])
    client = actionlib.SimpleActionClient('%s/homing'%target_motor, HomingAction)
    client.wait_for_server()

    goal = HomingActionGoal()
    client.send_goal(goal, done_cb=func_done, feedback_cb=func_feedback, active_cb=func_active)

    result = client.wait_for_result()
    rospy.loginfo(result)
    quit()


if __name__ == '__main__':
    rospy.init_node('request_homing_node', anonymous=False)
    m = main()
    rospy.spin()
