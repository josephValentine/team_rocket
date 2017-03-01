#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

import numpy as np

#### Global vars ####

_ctrl_period = 1.0/100

# Lowpass filter constants
# Need to be tuned
_alpha = 0.1
_beta  = 0.1

# Flag to indicate when a message is received
_msg_received = False

# Measured positions
_measured = Pose2D()

# Predicted positions
_hat = Pose2D()

# Predicted velocities
_vel = Twist()

# -------------------

def _handle(msg):
    global _measured, _msg_received
    _measured = msg
    _msg_received = True
    
    
def _estimate():
    global _hat, _vel, _msg_received
    if _msg_received:
        _vel.linear.x = _beta * _vel.linear.x + (1 - _beta) * (_measured.x - _hat.x) / _ctrl_period
        _vel.linear.y = _beta * _vel.linear.y + (1 - _beta) * (_measured.y - _hat.y) / _ctrl_period
        
        _hat.x = _hat.x + _ctrl_period * _vel.linear.x
        _hat.y = _hat.y + _ctrl_period * _vel.linear.y
        
        _hat.x = _alpha * _hat.x + (1 - _alpha) * _measured.x
        _hat.y = _alpha * _hat.y + (1 - _alpha) * _measured.y
        
        _msg_received = False
        
    else:
        _hat.x = _hat.x + _ctrl_period * _vel.linear.x
        _hat.y = _hat.y + _ctrl_period * _vel.linear.y

        
def main():
    rospy.init_node('estimator', anonymous=False)

    # Subscribe to the position of the object on the field (from vision)
    rospy.Subscriber('data', Pose2D, _handle)

    # Publish estimated states
    pub = rospy.Publisher('estimated_state', Pose2D, queue_size=10)

    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():

        _estimate()

        # Publish estimated states
        pub.publish(_hat)

        # Wait however long it takes to make this tick at proper control period
        rate.sleep()



if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()
