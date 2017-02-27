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

# _img = Image()

# _tmp_cnt = 0
# def _handle_img(msg):
   # global _tmp_cnt
   #print "handle_img"
   # _tmp_cnt = (_tmp_cnt + 1) % 100
   # if _tmp_cnt == 0:
   #    print 'msg:', dir(msg)
   # global _img

   

   # _img = _ros2cv(msg)
   # _show_raw(_ros2cv(msg))
   # _ourRobot1(_ros2cv(msg))
   # _ball(_ros2cv(msg))
   # _process_img(_ros2cv(msg))
   # _color_calibration(_ros2cv(msg))

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

#publish the raw coordinate data of the objects
def _process_img(msg):

   image = _ros2cv(msg)
   _show_raw(image)

    # publish locations
   us1_pub   = rospy.Publisher('us1', Pose2D, queue_size=10)
   # us2_pub   = rospy.Publisher('us2', Pose2D, queue_size=10)
   # them1_pub = rospy.Publisher('them1', Pose2D, queue_size=10)
   # them2_pub = rospy.Publisher('them2', Pose2D, queue_size=10)
   ball_pub  = rospy.Publisher('ball', Pose2D, queue_size=10)

   fieldColor=[200,200,200,50,100,50]
   ourRobot1Color=[229,216,241,204,195,222]
   ourRobot2Color=[229,216,241,204,195,222]
   ballColor=[246, 249, 236, 215, 220, 163]
   opponent1Color=[229,216,241,204,195,222]
   opponent2Color=[229,216,241,204,195,222]

   field = _field(image, fieldColor)
   # ourRobot1 = _ourRobot1(image, ourRobot1Color)
   # if ourRobot1:
   #    us1_msg = convert_coordinates(ourRobot1[0],ourRobot1[1],ourRobot1[2], field)
   #    us1_pub.publish(us1_msg)

   # ourRobot2=convert_coordinates(_ourRobot2(image, ourRobot2Color), field)
   # ball = _ball(image, ballColor)
   # if ball:
   #    ball_msg = convert_coordinates(ball[0], ball[1], ball[2], field)
   #    ball_pub.publish(ball_msg)
   
   
   

def convert_coordinates(x,y,theta, field):
   # This assumes the field position zero is top left
   # field param is (x,y,w,h)
   realW = 3.40
   scaleW = realW / field[2]
   realH = 2.38
   scaleH = realH / field[3]
   rval = Pose2D()
   rval.x = (x - field[0])*scaleW
   rval.y = (y - field[1])*scaleH
   rval.theta = theta
   return rval

def _color_mask(image, controlWindow, color):
   # create the filter color params
   cv2.createTrackbar('hueMax',controlWindow,color[0],255,_nothing)
   cv2.createTrackbar('satMax',controlWindow,color[1],255,_nothing)
   cv2.createTrackbar('volMax',controlWindow,color[2],255,_nothing)
   cv2.createTrackbar('hueMin',controlWindow,color[3],255,_nothing)
   cv2.createTrackbar('satMin',controlWindow,color[4],255,_nothing)
   cv2.createTrackbar('volMin',controlWindow,color[5],255,_nothing)
   # loop over the boundaries
   
   # create NumPy arrays from the boundaries
   hueMax = cv2.getTrackbarPos('hueMax',controlWindow)
   satMax = cv2.getTrackbarPos('satMax',controlWindow)
   volMax = cv2.getTrackbarPos('volMax',controlWindow)
   hueMin = cv2.getTrackbarPos('hueMin',controlWindow)
   satMin = cv2.getTrackbarPos('satMin',controlWindow)
   volMin = cv2.getTrackbarPos('volMin',controlWindow)

   lower= [volMin, satMin, hueMin]
   upper = [volMax, satMax, hueMax]
   lower = np.array(lower, dtype = "uint8")
   upper = np.array(upper, dtype = "uint8")

   #resize the image to reduce process time, fx and fy are the scale factors in those axis
   image = cv2.resize(image, (0,0), fx=0.7, fy=0.7) 

   # find the colors within the specified boundaries and apply
   # the mask
   mask = cv2.inRange(image, lower, upper)
   out1 = cv2.bitwise_and(image, image, mask = mask)
   return out1

def _ourRobot1(image, color):

   cv2.namedWindow("ourRobot1")
   rate = rospy.Rate(100)

   out1 = _color_mask(image,'ourRobot1',color)
   #remove noise
   kernel = np.ones((3,3),np.uint8) #sets size of holes accepted i think?
   out2 = cv2.morphologyEx(out1, cv2.MORPH_OPEN, kernel)
   out3 = cv2.morphologyEx(out2, cv2.MORPH_CLOSE, kernel)

   #find contours of objects
   out4= cv2.cvtColor(out3, cv2.COLOR_BGR2GRAY)
   # ret,thresh = cv2.threshold(out4,127,255,0)
   (_,cnts, _) = cv2.findContours(out4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

   objects=[]
   for c in cnts:
      pt,radius = cv2.minEnclosingCircle(c)
      # cv2.drawContours(out4, [c],0,(0,128,255),1)
      objects.append((pt[0],pt[1],radius))
  
   if len(objects) >= 2:
      sortedObj = sorted(objects,key=lambda x: x[2])
      angle = np.arctan2((sortedObj[1][1]-sortedObj[0][1]), (sortedObj[1][0]-sortedObj[0][0]))*180/np.pi + 180
      x = (sortedObj[0][0]+sortedObj[1][0])/2
      y = (sortedObj[0][1]+sortedObj[1][1])/2
      print(x,y,angle)
      cv2.circle(out3, (int(sortedObj[0][0]), int(sortedObj[0][1])), int(sortedObj[0][2]), (0,0,255), -1)
      cv2.circle(out3, (int(sortedObj[1][0]), int(sortedObj[1][1])), int(sortedObj[1][2]), (255,0,0), -1)
      return (x,y,angle)

   # # show the images
   # cv2.imshow("Control", np.hstack([image, out3]))
   cv2.imshow("ourRobot1",out3)
   cv2.waitKey(2)

def _field(image, color):

   cv2.namedWindow("field")
   # rate = rospy.Rate(100)

   out1 = _color_mask(image, 'field',color)
   #remove noise
   kernel = np.ones((30,30),np.uint8) #sets size of holes accepted i think?
   out2 = cv2.morphologyEx(out1, cv2.MORPH_OPEN, kernel)
   out3 = cv2.morphologyEx(out2, cv2.MORPH_CLOSE, kernel)

   #find contours of objects
   out4= cv2.cvtColor(out3, cv2.COLOR_BGR2GRAY)
   # ret,thresh = cv2.threshold(out4,127,255,0)
   (_,cnts, _) = cv2.findContours(out4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

   objects=[]
   for c in cnts:
      x,y,w,h = cv2.boundingRect(c)
      # cv2.drawContours(out4, [c],0,(0,128,255),1)
      objects.append((x,y,w,h))
  
   if len(objects) >= 1:
      sortedObj = sorted(objects,key=lambda x: x[2])
      x = sortedObj[len(objects) - 1][0]
      y = sortedObj[len(objects) - 1][1]
      w = sortedObj[len(objects) - 1][2]
      h = sortedObj[len(objects) - 1][3]
      # print(x,y,w,h)
      cv2.rectangle(out3, (int(x), int(y)), (int(x+w), int(y+h)), (0,255,0), 2)
      return (x,y,w,h)
   # # show the images
   # cv2.imshow("Control", np.hstack([image, out3]))
   cv2.imshow("field",out3)
   # cv2.waitKey(2)

def _ball(image, color):

   cv2.namedWindow("ball")
   rate = rospy.Rate(100)

   out1 = _color_mask(image, 'ball',color)
   #remove noise
   kernel = np.ones((3,3),np.uint8) #sets size of holes accepted i think?
   out2 = cv2.morphologyEx(out1, cv2.MORPH_OPEN, kernel)
   out3 = cv2.morphologyEx(out2, cv2.MORPH_CLOSE, kernel)

   #find contours of objects
   out4= cv2.cvtColor(out3, cv2.COLOR_BGR2GRAY)
   # ret,thresh = cv2.threshold(out4,127,255,0)
   (_,cnts, _) = cv2.findContours(out4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

   objects=[]
   for c in cnts:
      pt,radius = cv2.minEnclosingCircle(c)
      # cv2.drawContours(out4, [c],0,(0,128,255),1)
      objects.append((pt[0],pt[1],radius))
  
   if len(objects) >= 1:
      sortedObj=sorted(objects,key=lambda x: x[2])
      x=sortedObj[0][0]
      y=sortedObj[0][1]
      # print('ball',x,y)
      cv2.circle(out3, (int(x), int(y)), int(sortedObj[0][2]), (100,100,255), -1)
      return (x,y,0)
   
   cv2.imshow("ball",out3)
   cv2.waitKey(2)

def main():
   rospy.init_node('ai', anonymous=False)

   # subscribe to camera
   rospy.Subscriber('camera', Image, _process_img)
   
   # us1_pub   = rospy.Publisher('us1', Pose2D, queue_size=10)
   # us2_pub   = rospy.Publisher('us2', Pose2D, queue_size=10)
   # them1_pub = rospy.Publisher('them1', Pose2D, queue_size=10)
   # them2_pub = rospy.Publisher('them2', Pose2D, queue_size=10)
   # ball_pub  = rospy.Publisher('ball', Pose2D, queue_size=10)
   
   rate = rospy.Rate(100) # 100 Hz
   while not rospy.is_shutdown():

      # Get a message ready to send
      # us1_msg   = Pose2D()
      # us2_msg   = Pose2D()
      # them1_msg = Pose2D()
      # them2_msg = Pose2D()
      # ball_msg  = Pose2D()

      # # do vision stuff
      # us1_msg.x, us1_msg.y, us1_msg.theta       = -0.25, 0.0, 0.0
      # us2_msg.x, us2_msg.y, us2_msg.theta       = -0.5, 0.0, 0.0
      # them1_msg.x, them1_msg.y, them1_msg.theta = 0.25, 0.0, 180.0
      # them2_msg.x, them2_msg.y, them2_msg.theta = 0.5, 0.0, 180.0
      # ball_msg.x, ball_msg.y, ball_msg.theta    = 0.0, 0.0, 0.0

      # # publish
      # us1_pub.publish(us1_msg)
      # us2_pub.publish(us2_msg)
      # them1_pub.publish(them1_msg)
      # them2_pub.publish(them2_msg)
      # ball_pub.publish(ball_msg)

      # Wait however long it takes to make this tick at 100Hz
      rate.sleep()

if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()
