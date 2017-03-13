#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

import numpy as np

import Controller

#### Global vars ####

_ctrl_period = 1.0/100

# Flag to indicate when a message is received from vision
_msg_received = False

# My measured state
_xmeas = 0
_ymeas = 0
_thetameas = 0

# My estimated state
_xhat = 0
_yhat = 0
_thetahat = 0

# Lowpass filter constants
# Need to be tuned
_alpha = 0.9
_beta  = 0.9

# -------------------

def _handle_me(msg):
    # print 'Controller: _handle_me'
    global _xmeas, _ymeas, _thetameas, _msg_received
    # print 'xhat: {}\nyhat: {}\nthetahat: {}'.format(
    #     _xhat, _yhat, _thetahat)
    # print 'Controller: _handle_me'
    # print '\tme: {}'.format(msg)
    _xmeas = msg.x
    _ymeas = msg.y
    _thetameas = msg.theta
    _msg_received = True


# See Constants.py
field_length        = 3.40 # (134 in)  ## Old field was 3.68 # (12ft)
field_width         = 2.38 # (93.875 in) ## Old field was 2.62 # (8.58 ft)
def _handle_desired_position(msg):
    # print 'Controller: _handle_desired_position'
    # print '\tdesired_pos: {}'.format(msg)
    # Controller.set_commanded_position(msg.x, msg.y, msg.theta)
    Controller.set_commanded_position(field_length/2, field_width/2, 0)
    print 'commanded x = {}\ncommanded y = {}\n'.format(field_length/2, field_width/2)


def _estimate_state(vx, vy, w):
    # print 'Controller: estimating state'
    global _xhat, _yhat, _thetahat, _msg_received, _alpha
    
    _xhat = _xhat + _ctrl_period * vx
    _yhat = _yhat + _ctrl_period * vy
    _thetahat = _thetahat + _ctrl_period * w
    
    if _msg_received:
        # print '\tmessage received'
        _xhat = _alpha * _xhat + (1 - _alpha) * _xmeas
        _yhat = _alpha * _yhat + (1 - _alpha) * _ymeas
        _thetahat = _alpha * _thetahat + (1 - _alpha) * _thetameas
        
        _msg_received = False


def main():
    rospy.init_node('controller', anonymous=False)

    # Subscribe to my measured state (from the vision node)
    # and my desired state (from the ai node)
    rospy.Subscriber('me', Pose2D, _handle_me)
    rospy.Subscriber('desired_position', Pose2D, _handle_desired_position)

    # Publish velocity commands from PID controller
    # and the estimated state of the robot
    pub = rospy.Publisher('vel_cmds', Twist, queue_size=10)
    pubState = rospy.Publisher('estimated_state', Pose2D, queue_size=10)

    # initialize the controller
    Controller.init()
    
    # Initialize commanded velocities
    vx = 0
    vy = 0
    w = 0

    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():
        
        _estimate_state(vx, vy, w)

        # (vx, vy, w) = Controller.update(_ctrl_period, _xmeas, _ymeas, _thetameas)
        (vx, vy, w) = Controller.update(_ctrl_period, _xhat, _yhat, _thetahat)


        # print 'vx: {}\nvy: {}\nw: {}'.format(vx, vy, w)
        # Publish Velocity Commands
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w #this is wierd, i assume this is supposed to be angular velocity but it seems to be spitting out
        # staying at same angle, i.e. angular velocity of 0
        # cont_factor = 5
        # msg.linear.x, msg.linear.y = vx*cont_factor, vy*cont_factor
        # msg.angular.z = 0
        # print 'Controller Before: vx: {:.3f}\tvy: {:.3f}\t w: {:.3f}'.format(vx,vy,w)
        # print 'Controller Twist:  vx: {:.3f}\tvy: {:.3f}\t w: {:.3f}'.format(
        #     msg.linear.x,msg.linear.y,msg.angular.z)
        pub.publish(msg)
        
        # Publish estimated state of robot
        state = Pose2D()
        state.x = _xhat
        state.y = _yhat
        state.theta = _thetahat
        # right now the estimator isn't working, using measured values
        # state.x = _xmeas
        # state.y = _ymeas
        # state.theta = _thetameas
        pubState.publish(state)

        # Wait however long it takes to make this tick at proper control period
        rate.sleep()



if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()
