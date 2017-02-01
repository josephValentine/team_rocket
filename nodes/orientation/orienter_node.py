#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D
# from std_srvs.srv import Trigger, TriggerResponse

_team_side = 'home'
_home_1 = Pose2D()
_home_2 = Pose2D()
_away_1 = Pose2D()
_away_2 = Pose2D()

def _handle_home_1(msg):
   _home_1 = msg

def _handle_home_2(msg):
   _home_2 = msg

def _handle_away_1(msg):
   _away_1 = msg

def _handle_away_2(msg):
   _away_2 = msg


def main():
    global _team_side
    param_name = rospy.search_param('team_side')
    _team_side = rospy.get_param(param_name, 'home')

    rospy.init_node('orienter', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('vision/home1', Pose2D, _handle_home_1)
    rospy.Subscriber('vision/home2', Pose2D, _handle_home_2)
    rospy.Subscriber('vision/away1', Pose2D, _handle_away_1)
    rospy.Subscriber('vision/away2', Pose2D, _handle_away_2)

    pub_us_1 = rospy.Publisher('orienter/us1', Pose2D, queue_size=10)
    pub_us_2 = rospy.Publisher('orienter/us2', Pose2D, queue_size=10)
    pub_them_1 = rospy.Publisher('orienter/them1', Pose2D, queue_size=10)
    pub_them_2 = rospy.Publisher('orienter/them2', Pose2D, queue_size=10)

    ctrl_period = 100
    
    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():

         if _team_side == 'home':
            # publish
            pub_us_1.publish(_home_1)
            pub_us_2.publish(_home_2)
            pub_them_1.publish(_away_1)
            pub_them_2.publish(_away_2)
         else:
            # flip coordinate system
            for msg in [_home_1, _home_2, _away_1, _away_2]:
               msg.x = -msg.x
               msg.y = -msg.y
               msg.theta = msg.theta + 180 % 360

            # publish
            pub_us_1.publish(_away_1)
            pub_us_2.publish(_away_2)
            pub_them_1.publish(_home_1)
            pub_them_2.publish(_home_2)


        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
