#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image

#### Global vars ####

img = Image()

_tmp_cnt = 0
def _handle_img(msg):
   global _tmp_cnt
   print "handle_img"
   _tmp_cnt = (_tmp_cnt + 1) % 100
   if _tmp_cnt == 0:
      print 'msg:', dir(msg)
   global _img
   _img = msg


def main():
   rospy.init_node('ai', anonymous=False)

   # subscribe to camera
   rospy.Subscriber('camera', Image, _handle_img)

   # publish locations
   us1_pub   = rospy.Publisher('us1', Pose2D, queue_size=10)
   us2_pub   = rospy.Publisher('us2', Pose2D, queue_size=10)
   them1_pub = rospy.Publisher('them1', Pose2D, queue_size=10)
   them2_pub = rospy.Publisher('them2', Pose2D, queue_size=10)
   ball_pub  = rospy.Publisher('ball', Pose2D, queue_size=10)

   rate = rospy.Rate(100) # 100 Hz
   while not rospy.is_shutdown():

      # Get a message ready to send
      us1_msg   = Pose2D()
      us2_msg   = Pose2D()
      them1_msg = Pose2D()
      them2_msg = Pose2D()
      ball_msg  = Pose2D()

      # do vision stuff
      us1_msg.x, us1_msg.y, us1_msg.theta       = -0.25, 0.0, 0.0
      us2_msg.x, us2_msg.y, us2_msg.theta       = -0.5, 0.0, 0.0
      them1_msg.x, them1_msg.y, them1_msg.theta = 0.25, 0.0, 180.0
      them2_msg.x, them2_msg.y, them2_msg.theta = 0.5, 0.0, 180.0
      ball_msg.x, ball_msg.y, ball_msg.theta    = 0.0, 0.0, 0.0

      # publish
      us1_pub.publish(us1_msg)
      us2_pub.publish(us2_msg)
      them1_pub.publish(them1_msg)
      them2_pub.publish(them2_msg)
      ball_pub.publish(ball_msg)

      # Wait however long it takes to make this tick at 100Hz
      rate.sleep()

if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()
