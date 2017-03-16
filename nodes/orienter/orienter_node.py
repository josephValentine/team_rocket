#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D

# Rate
_rate = 100

# Team Side (default = home)
_home_side = 'home'
_away_side = 'away'
_team_side = _home_side

# # Keys
# _home1_key = 'home1'
# _home2_key = 'home2'
# _away1_key = 'away1'
# _away2_key = 'away2'
# _ball_key  = 'ball'

# _all_keys = [_home1_key, _home2_key, _away1_key, _away2_key, _ball_key]

# # Messages
# _home1_msg = Pose2D()
# _home2_msg = Pose2D()
# _away1_msg = Pose2D()
# _away2_msg = Pose2D()
# _ball_msg  = Pose2D()

# _all_msgs = {_home1_key : _home1_msg, _home2_key : _home2_msg,
#              _away1_key : _away1_msg, _away2_key : _away2_msg,
#              _ball_key  : _ball_msg}
             
# Publishers
_pub_us1 = rospy.Publisher('orienter/us1', Pose2D, queue_size=10)
_pub_us2 = rospy.Publisher('orienter/us2', Pose2D, queue_size=10)
_pub_them1 = rospy.Publisher('orienter/them1', Pose2D, queue_size=10)
_pub_them2 = rospy.Publisher('orienter/them2', Pose2D, queue_size=10)
_pub_ball  = rospy.Publisher('orienter/ball',  Pose2D, queue_size=10)


def _flip_coordinates(data):
    oriented = data
    # camera is already flipped...so we don't actually need this node
    # if _team_side == _away_side:
    #     oriented.x = -oriented.x
    #     oriented.y = -oriented.y
    #     oriented.theta = (oriented.theta + 180) % 360
    return oriented
    

# Handlers
def _handle_us1(msg):
    oriented = _flip_coordinates(msg)
    _pub_us1.publish(oriented)


def _handle_us2(msg):
    oriented = _flip_coordinates(msg)
    _pub_us2.publish(oriented)


def _handle_them1(msg):
    oriented = _flip_coordinates(msg)
    _pub_them1.publish(oriented)


def _handle_them2(msg):
    oriented = _flip_coordinates(msg)
    _pub_them2.publish(oriented)


def _handle_ball(msg):
    oriented = _flip_coordinates(msg)
    _pub_ball.publish(oriented)


def main():
    global _team_side
    param_name = rospy.search_param('team_side')
    # get which team you're on, default to home
    _team_side = rospy.get_param(param_name, _home_side)

    rospy.init_node('orienter', anonymous=False)

    # Subscribers
    rospy.Subscriber('vision/us1', Pose2D, _handle_us1)
    rospy.Subscriber('vision/us2', Pose2D, _handle_us2)
    rospy.Subscriber('vision/them1', Pose2D, _handle_them1)
    rospy.Subscriber('vision/them2', Pose2D, _handle_them2)
    rospy.Subscriber('vision/ball',  Pose2D, _handle_ball)

    # all_pubs = {_home1_key : pub_home1, _home2_key : pub_home2,
    #             _away1_key : pub_away1, _away2_key : pub_away2,
    #             _ball_key  : pub_ball}

    rate = rospy.Rate(int(_rate))
    while not rospy.is_shutdown():

        # # Flip coordinate system if necessary
        # if _team_side == 'away':
        #     for msg in _all_msgs:
        #         msg.x = -msg.x
        #         msg.y = -msg.y
        #         msg.theta = (msg.theta + 180) % 360

        # # Publish Oriented Coordinates
        # for key in _all_keys:
        #     msg = _all_msgs[key]
        #     pub = all_pubs[key]
        #     pub.publish(msg)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
