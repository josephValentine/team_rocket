#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D

# Team Side (default = home)
_team_side = 'home'

# Keys
_home1_key = 'home1'
_home2_key = 'home2'
_away1_key = 'away1'
_away2_key = 'away2'
_ball_key  = 'ball'

_all_keys = [_home1_key, _home2_key, _away1_key, _away2_key, _ball_key]

# Messages
_home1_msg = Pose2D()
_home2_msg = Pose2D()
_away1_msg = Pose2D()
_away2_msg = Pose2D()
_ball_msg  = Pose2D()

_all_msgs = {_home1_key : _home1_msg, _home2_key : _home2_msg,
             _away1_key : _away1_msg, _away2_key : _away2_msg,
             _ball_key  : _ball_msg}

# Handlers
def _handle_home1(msg):
    global _home1_msg
    _home1_msg = msg


def _handle_home2(msg):
    global _home2_msg
    _home2_msg = msg


def _handle_away1(msg):
    global _away1_msg
    _away1_msg = msg


def _handle_away2(msg):
    global _away2_msg
    _away2_msg = msg


def _handle_ball(msg):
    global _ball_msg
    _ball_msg = msg


def main():
    global _team_side
    param_name = rospy.search_param('team_side')
    # get which team you're on, default to home
    _team_side = rospy.get_param(param_name, 'home')

    rospy.init_node('orienter', anonymous=False)

    # Subscribers
    rospy.Subscriber('vision/home1', Pose2D, _handle_home1)
    rospy.Subscriber('vision/home2', Pose2D, _handle_home2)
    rospy.Subscriber('vision/away1', Pose2D, _handle_away1)
    rospy.Subscriber('vision/away2', Pose2D, _handle_away2)
    rospy.Subscriber('vision/ball',  Pose2D, _handle_ball)

    # Publishers
    pub_home1 = rospy.Publisher('orienter/home1', Pose2D, queue_size=10)
    pub_home2 = rospy.Publisher('orienter/home2', Pose2D, queue_size=10)
    pub_away1 = rospy.Publisher('orienter/away1', Pose2D, queue_size=10)
    pub_away2 = rospy.Publisher('orienter/away2', Pose2D, queue_size=10)
    pub_ball  = rospy.Publisher('orienter/ball',  Pose2D, queue_size=10)
    all_pubs = {_home1_key : pub_home1, _home2_key : pub_home2,
                _away1_key : pub_away1, _away2_key : pub_away2,
                _ball_key  : pub_ball}

    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():

        # Flip coordinate system if necessary
        if _team_side == 'away':
            for msg in _all_msgs:
                msg.x = -msg.x
                msg.y = -msg.y
                msg.theta = (msg.theta + 180) % 360

        # Publish Oriented Coordinates
        for key in _all_keys:
            msg = _all_msgs[key]
            pub = all_pubs[key]
            pub.publish(msg)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
