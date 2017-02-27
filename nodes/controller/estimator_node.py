#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

import numpy as np

#### Global vars ####

_ctrl_period = 1.0/100

# Lowpass filter constants
_alpha = 0.1
_beta  = 0.1

# Flag to indicate when a message is received
_msg_received_us1 = False
_msg_received_us2 = False
_msg_received_them1 = False
_msg_received_them2 = False
_msg_received_ball = False

# Measured positions
_us1_measured = Pose2D()
_us2_measured = Pose2D()
_them1_measured = Pose2D()
_them2_measured = Pose2D()
_ball_measured = Pose2D()

# Predicted positions
_us1_hat = Pose2D()
_us2_hat = Pose2D()
_them1_hat = Pose2D()
_them2_hat = Pose2D()
_ball_hat = Pose2D()

# Predicted velocities
_us1_vel = Twist()
_us2_vel = Twist()
_them1_vel = Twist()
_them2_vel = Twist()
_ball_vel = Twist()

# -------------------

def _handle_us1(msg):
    global _us1_measured, _msg_received_us1
    _us1_measured = msg
    _msg_received_us1 = True
    
def _handle_us2(msg):
    global _us2_measured, _msg_received_us2
    _us2_measured = msg
    _msg_received_us2 = True
    
def _handle_them1(msg):
    global _them1_measured, _msg_received_them1
    _them1_measured = msg
    _msg_received_them1 = True
    
def _handle_them2(msg):
    global _them2_measured, _msg_received_them2
    _them2_measured = msg
    _msg_received_them2 = True
    
def _handle_ball(msg):
    global _ball_measured, _msg_received_ball
    _ball_measured = msg
    _msg_received_ball = True
    
def _estimate_opponent1():
    global _them1_hat, _them1_vel, _msg_received_them1
    if _msg_received_them1:
        _them1_vel.linear.x = _beta * _them1_vel.linear.x + (1 - _beta) * (_them1_measured.x - _them1_hat.x) / _ctrl_period
        _them1_vel.linear.y = _beta * _them1_vel.linear.y + (1 - _beta) * (_them1_measured.y - _them1_hat.y) / _ctrl_period
        
        _them1_hat.x = _them1_hat.x + _ctrl_period * _them1_vel.linear.x
        _them1_hat.y = _them1_hat.y + _ctrl_period * _them1_vel.linear.y
        
        _them1_hat.x = _alpha * _them1_hat.x + (1 - _alpha) * _them1_measured.x
        _them1_hat.y = _alpha * _them1_hat.y + (1 - _alpha) * _them1_measured.y
        
        _msg_received_them1 = False
        
    else:
        _them1_hat.x = _them1_hat.x + _ctrl_period * _them1_vel.linear.x
        _them1_hat.y = _them1_hat.y + _ctrl_period * _them1_vel.linear.y
        

def _estimate_opponent2():
    global _them2_hat, _them2_vel, _msg_received_them2
    if _msg_received_them2:
        _them2_vel.linear.x = _beta * _them2_vel.linear.x + (1 - _beta) * (_them2_measured.x - _them2_hat.x) / _ctrl_period
        _them2_vel.linear.y = _beta * _them2_vel.linear.y + (1 - _beta) * (_them2_measured.y - _them2_hat.y) / _ctrl_period
        
        _them2_hat.x = _them2_hat.x + _ctrl_period * _them2_vel.linear.x
        _them2_hat.y = _them2_hat.y + _ctrl_period * _them2_vel.linear.y
        
        _them2_hat.x = _alpha * _them2_hat.x + (1 - _alpha) * _them2_measured.x
        _them2_hat.y = _alpha * _them2_hat.y + (1 - _alpha) * _them2_measured.y
        
        _msg_received_them2 = False
        
    else:
        _them2_hat.x = _them2_hat.x + _ctrl_period * _them2_vel.linear.x
        _them2_hat.y = _them2_hat.y + _ctrl_period * _them2_vel.linear.y
        
        
def main():
    rospy.init_node('estimator', anonymous=False)

    # Subscribe to the positions of all the objects on the field (from vision)
    rospy.Subscriber('us1', Pose2D, _handle_us1)
    rospy.Subscriber('us2', Pose2D, _handle_us2)
    rospy.Subscriber('them1', Pose2D, _handle_them1)
    rospy.Subscriber('them2', Pose2D, _handle_them2)
    rospy.Subscriber('ball', Pose2D, _handle_ball)

    # Publish estimated states
    


    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():

        _estimate_opponent1()
        _estimate_opponent2()

        # Publish estimated states
        

        # Wait however long it takes to make this tick at proper control period
        rate.sleep()



if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()
