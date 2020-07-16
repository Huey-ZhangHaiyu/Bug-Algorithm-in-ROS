#! /usr/bin/env python
"""
bug1 node
Author:Haiyu Zhang
Date:12/13/2019
"""
"""
This program allows the robot use bug1 algorithm to reach to the goal point
please read the readme.txt to find how to test this program
"""
"""
state0:Go to point
state1:circumnavigate obstacle
state2:go to the closest point
"""
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'circumnavigate obstacle', 'go to closest point']
state_ = 0
circumnavigate_starting_point_ = Point()
circumnavigate_closest_point_ = Point()
count_state_time_ = 0  # seconds the robot is in a state
count_loop_ = 0


def clbk_odom(msg):
    global position_, yaw_

    position_ = msg.pose.pose.position
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(msg.ranges[255:295]),
        'front':  min(msg.ranges[0:5]+msg.ranges[355:360]),
        'left':   min(msg.ranges[15:60]),
    }


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)


def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
    return dist


def normalize_angle(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global circumnavigate_closest_point_, circumnavigate_starting_point_
    global count_loop_, count_state_time_
    
    rospy.init_node('bug1')
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    
    # initialize going to the point
    change_state(0)
    
    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if regions_ is None:
            continue
        
        if state_ == 0:  # go to point, if detects a wall, wall following
            if 0.1 < regions_['front'] < 0.2:
                circumnavigate_closest_point_ = position_  # the closest point is start point
                circumnavigate_starting_point_ = position_
                change_state(1)
        
        elif state_ == 1:
            # if current position is closer to the goal, assign current position to closest_point
            if calc_dist_points(position_, desired_position_) < calc_dist_points(circumnavigate_closest_point_, desired_position_):
                circumnavigate_closest_point_ = position_
                
            # compare only after 5 seconds - need some time to get out of starting_point
            # if robot reaches (is close to) starting point
            if count_state_time_ > 5 and \
               calc_dist_points(position_, circumnavigate_starting_point_) < 0.2:
                change_state(2)
        
        elif state_ == 2:
            # reach to the closest point
            if calc_dist_points(position_, circumnavigate_closest_point_) < 0.2:
                change_state(0)
                
        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        rate.sleep()


if __name__ == "__main__":
    main()
