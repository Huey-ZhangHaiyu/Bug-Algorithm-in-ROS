#! /usr/bin/env python
"""
follow_the_wall node
Author:Haiyu Zhang
Date:12/13/2019
"""
"""
This program allows the robot to follow the wall
this program can not find the wall in the initial position
you need to move it to close to the wall to start the program
please read the readme.txt to find how to test this program alone
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
import math

active_ = False  # False default
pub_ = None
regions_ = {'right': 0,'front': 0,'left': 0}
state_ = 0
state_dict_ = {0: 'find the wall',1: 'fleft',2: 'follow the wall',3: 'fright',4: 'avoid'}


def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
    global regions_
    regions_ = {'right':  min(msg.ranges[285:310]),
                'front':  min(msg.ranges[0:5]+msg.ranges[355:360]),
                'left':   min(msg.ranges[15:60])}
    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    regions = regions_
    if regions['front'] > 0.25:  # detect no wall front
        if regions['right'] < 0.15:  # too close to the wall
            change_state(1)  # change state to 1, turn left
        elif 0.15 <= regions['right'] <= 0.25:
            change_state(2)  # change state to 2, straight
        else:  # too far to the wall
            change_state(3)  # change state to 3, turn right
    else:  # detect wall front
        change_state(4)  # change state to 4, avoid the wall


def find_wall():
    global regions_
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0
    return msg


def turn_left():
    global regions_
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = 0.6
    return msg


def turn_right():
    global regions_
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -1.0
    return msg


def avoid():
    global regions_
    msg = Twist()
    msg.angular.z = 1.2
    msg.linear.x = 0
    return msg


def follow_the_wall():
    global regions_
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0.1
    return msg


def main():
    global pub_, active_
    
    rospy.init_node('follow_wall')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        elif state_ == 3:
            msg = turn_right()
        elif state_ == 4:
            msg = avoid()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
