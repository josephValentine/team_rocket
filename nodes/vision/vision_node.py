#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import String

#### Global vars ####

_img = Image()

_tmp_cnt = 0
def _handle_img(msg):
   global _tmp_cnt
   #print "handle_img"
   _tmp_cnt = (_tmp_cnt + 1) % 100
   if _tmp_cnt == 0:
      print 'msg:', dir(msg)
   global _img
   _img = _ros2cv(msg)
   #_show_raw(_ros2cv(msg))
   _show_filter_rt(_ros2cv(msg))

def _ros2cv(msg):
   try:
      cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
      return cv_image
   except CvBridgeError as e:
      print(e)

def _show_raw(cv_image):
   cv2.imshow("Image window", cv_image)
   cv2.waitKey(3)

def _nothing(x):
    pass


def _show_filter_rt(image):
  # define the list of boundaries
   cv2.namedWindow("Control")
   rate = rospy.Rate(100)
   # loop over the boundaries

   # create NumPy arrays from the boundaries
   Rup = 116
   Gup = 236
   Bup = 235
   Rdn = 50
   Gdn = 158
   Bdn = 158
   lower= [Bdn, Gdn, Rdn]
   upper = [Bup, Gup, Rup]
   lower = np.array(lower, dtype = "uint8")
   upper = np.array(upper, dtype = "uint8")

   # find the colors within the specified boundaries and apply
   # the mask
   mask = cv2.inRange(image, lower, upper)
   output = cv2.bitwise_and(image, image, mask = mask)

   # show the images

   cv2.imshow("Control", np.hstack([image, output]))

   cv2.waitKey(2)

def _show_filter(image):
  # define the list of boundaries
   boundaries = [
      #([17, 15, 100], [50, 56, 200]),
      #([86, 31, 4], [220, 88, 50]),
      #([25, 146, 190], [62, 174, 250]),
      ([110, 50, 50], [130, 255, 255]),
      ([103, 86, 65], [145, 133, 128])
   ]
   cv2.namedWindow("Control")

   cv2.createTrackbar('Rup','Control',0,255,_nothing)
   cv2.createTrackbar('Gup','Control',0,255,_nothing)
   cv2.createTrackbar('Bup','Control',0,255,_nothing)
   cv2.createTrackbar('Rdn','Control',0,255,_nothing)
   cv2.createTrackbar('Gdn','Control',0,255,_nothing)
   cv2.createTrackbar('Bdn','Control',0,255,_nothing)
   # loop over the boundaries
   while(1):
      # create NumPy arrays from the boundaries
      Rup = cv2.getTrackbarPos('Rup','Control')
      Gup = cv2.getTrackbarPos('Gup','Control')
      Bup = cv2.getTrackbarPos('Bup','Control')
      Rdn = cv2.getTrackbarPos('Rdn','Control')
      Gdn = cv2.getTrackbarPos('Gdn','Control')
      Bdn = cv2.getTrackbarPos('Bdn','Control')
      lower= [Bdn, Gdn, Rdn]
      upper = [Bup, Gup, Rup]
      lower = np.array(lower, dtype = "uint8")
      upper = np.array(upper, dtype = "uint8")

         # find the colors within the specified boundaries and apply
         # the mask
      mask = cv2.inRange(image, lower, upper)
      output = cv2.bitwise_and(image, image, mask = mask)

      # show the images

      cv2.imshow("Control", np.hstack([image, output]))

      cv2.waitKey(0)


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
