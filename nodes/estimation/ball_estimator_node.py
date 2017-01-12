#!/usr/bin/env python
import time

import rospy
from geometry_msgs.msg import Pose2D
from mcthuggets.msg import BallState

import numpy as np

"""
You could do state estimation here for the ball. For now,
this node's job is simply to take the Pose2D vision message
and patch it through as a BallState message.
"""

_state_pub = None
_team_side = None

# Again, this could be done in the vision code
from soccerref.msg import GameState
_game_state = GameState

def _handle_game_state(msg):
    global _game_state
    _game_state = msg

def _handle_vision_ball_position(msg):
    # Flip the coordinate system so that our side is always the negative
    # side. This is nice because we won't have to think about this later
    # in our high-level AI code and elsewhere.
    # This could also be done in the vision node; see original mcthuggets code
    flip = -1 if (_team_side !='home') ^ bool(_game_state.second_half) else 1

    # Construct ball_state message, BallState
    new_msg = BallState()
    new_msg.vision_x = new_msg.xhat = new_msg.xhat_future = flip*msg.x
    new_msg.vision_y = new_msg.yhat = new_msg.yhat_future = flip*msg.y
    new_msg.vx = 0
    new_msg.vy = 0
    new_msg.predict_forward_seconds = 0
    new_msg.correction = True # actual measurement
    _state_pub.publish(new_msg)

def main():
    rospy.init_node('ball_estimator', anonymous=False)

    global _team_side
    param_name = rospy.search_param('team_side')
    _team_side = rospy.get_param(param_name, 'home')

    # Sub/Pub
    global _state_pub
    _state_pub = rospy.Publisher('ball_state', BallState, queue_size=10)
    
    rospy.Subscriber('vision_position', Pose2D, _handle_vision_ball_position)
    rospy.Subscriber('/game_state', GameState, _handle_game_state)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
