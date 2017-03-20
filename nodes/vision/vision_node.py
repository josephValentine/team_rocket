#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import String

# change this if we're live (on the pi)
_live = False

def _ros2cv(msg):
   try:
      cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
      return cv_image
   except CvBridgeError as e:
      print(e)

def _show_raw(cv_image):
    # create a CLAHE object (Arguments are optional).
   clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
   b, g, r = cv2.split(cv_image)
   clb = clahe.apply(b)
   clg = clahe.apply(g)
   clr = clahe.apply(r)
   img=cv2.merge((clb, clg, clr))
   if not _live:
      cv2.imshow("Control", np.hstack([cv_image, img]))
   # cv2.imshow("Image window", cv_image)

def _nothing(x):
    pass

#publish the raw coordinate data of the objects
isFirst = True
def _process_img(msg):

   global isFirst
   # print 'hello everyone'
   # return

   image = _ros2cv(msg)
   # _show_raw(image)

   fieldColor=[189,108,215,123,25,31]
   ourRobot1Color=[255,89,128,159,54,98]
   ourRobot2Color=[229,216,241,204,195,222]
   ballColor=[255, 145, 238, 177, 6, 153]
   opponent1Color=[229,216,241,204,195,222]
   opponent2Color=[229,216,241,204,195,222]

   field = _field(image, fieldColor, isFirst)
   global us1_pub, us2_pub, ball_pub

   if all(field):
      ourRobot1 = _findRobot(image, ourRobot1Color, "ourRobot1", isFirst, (3,3))
      if (ourRobot1[0] is not None and ourRobot1[1] is not None):#angle can be zero so cant use all() func
         us1_msg = convert_coordinates(ourRobot1[0],ourRobot1[1],ourRobot1[2], field)
         us1_pub.publish(us1_msg)

      ourRobot2 = _findRobot(image, ourRobot2Color, "ourRobot2", isFirst, (3,3))
      if (ourRobot2[0] is not None and ourRobot2[1] is not None):#angle can be zero so cant use all() func
         us2_msg = convert_coordinates(ourRobot2[0],ourRobot2[1],ourRobot2[2], field)
         us2_pub.publish(us2_msg)
      
      ball = _ball(image, ballColor, isFirst)
      if (ball[0] is not None and ball[1] is not None):
         ball_msg = convert_coordinates(ball[0], ball[1], ball[2], field)
         ball_pub.publish(ball_msg)

   if not _live:
      cv2.waitKey(3)#the windows dont stay open if this isnt here...
   isFirst = False

   
   

def convert_coordinates(x,y,theta, field):
   # This assumes the field position zero is top left
   # field param is (x,y,w,h)
   # may need to rotate angles...
   realW = 3.40
   scaleW = realW / field[2]
   realH = 2.38
   scaleH = realH / field[3]
   rval = Pose2D()
   # subtract half the width/height to make 0,0 at center of field
   rval.x = (x - field[0])*scaleW - realW/2
   rval.y = (y - field[1])*scaleH - realH/2
   rval.theta = theta
   # flip y because image says y is down
   rval.y = -rval.y
   rval.theta = (-rval.theta) % 360
   return rval


def _color_mask(image, controlWindow, color, first=False):

   if not _live:
      # create the filter color params
      if first:
         print 'first for %s' % controlWindow
         first = False
         cv2.createTrackbar('hueMax',controlWindow,color[0],255,_nothing)
         cv2.createTrackbar('satMax',controlWindow,color[1],255,_nothing)
         cv2.createTrackbar('volMax',controlWindow,color[2],255,_nothing)
         cv2.createTrackbar('hueMin',controlWindow,color[3],255,_nothing)
         cv2.createTrackbar('satMin',controlWindow,color[4],255,_nothing)
         cv2.createTrackbar('volMin',controlWindow,color[5],255,_nothing)
      # # loop over the boundaries

      # # create NumPy arrays from the boundaries
      hueMax = cv2.getTrackbarPos('hueMax',controlWindow)
      satMax = cv2.getTrackbarPos('satMax',controlWindow)
      volMax = cv2.getTrackbarPos('volMax',controlWindow)
      hueMin = cv2.getTrackbarPos('hueMin',controlWindow)
      satMin = cv2.getTrackbarPos('satMin',controlWindow)
      volMin = cv2.getTrackbarPos('volMin',controlWindow)

   # if live
   else:
      hueMax = color[0]
      satMax = color[1]
      volMax = color[2]
      hueMin = color[3]
      satMin = color[4]
      volMin = color[5]

   lower= [volMin, satMin, hueMin]
   upper = [volMax, satMax, hueMax]
   lower = np.array(lower, dtype = "uint8")
   upper = np.array(upper, dtype = "uint8")

   #resize the image to reduce process time, fx and fy are the scale factors in those axis
   image = cv2.resize(image, (0,0), fx=0.7, fy=0.7) 

   # find the colors within the specified boundaries and apply
   # the mask
   hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
   mask = cv2.inRange(hsv, lower, upper)
   out1 = cv2.bitwise_and(image, image, mask = mask)

   return out1


def _findRobot(image, color, name, isFirst, size):
   """
   image: image
   color: array of color values for mask (6)
   name: name of object, window (string)
   isFirst: True if this is the first time (so we can initialize sliders)
   size: tuple of dimensions (2)

   """
   if not _live:
      cv2.namedWindow(name)
   out1 = _color_mask(image,name,color,isFirst)
   
   #remove noise
   kernel = np.ones(size,np.uint8) #sets size of holes accepted i think?
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
   
   x=None
   y=None
   angle=None
   if len(objects) >= 2:
      sortedObj = sorted(objects,key=lambda x: x[2],reverse=True)
      biggerObject, smallerObject = sortedObj[0], sortedObj[1]
      biggerX, biggerY = biggerObject[0], biggerObject[1]
      smallerX, smallerY = smallerObject[0], smallerObject[1]
      # go from bigger to smaller
      deltaX = smallerX - biggerX
      deltaY = smallerY - biggerY
      angle = (np.arctan2(deltaY, deltaX)*180/np.pi) % 360
      x = (biggerX+smallerX)/2
      y = (biggerY+smallerY)/2
      # print(x,y,angle)
      cv2.circle(out3, (int(sortedObj[0][0]), int(sortedObj[0][1])), int(sortedObj[0][2]), (0,0,255), 2)
      cv2.circle(out3, (int(sortedObj[1][0]), int(sortedObj[1][1])), int(sortedObj[1][2]), (255,0,0), 2)
      
   if not _live:
      cv2.imshow(name,out3)
   return (x,y,angle)


def _field(image, color, isFirst):

   if not _live:
      cv2.namedWindow("field")
   out1 = _color_mask(image, 'field',color,isFirst)
   
   #remove noise
   kernel = np.ones((10,10),np.uint8) #sets size of holes accepted i think?
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
  
   x=None
   y=None
   w=None
   h=None
   if len(objects) >= 1:
      sortedObj = sorted(objects,key=lambda x: x[2])
      x = sortedObj[len(objects) - 1][0]
      y = sortedObj[len(objects) - 1][1]
      w = sortedObj[len(objects) - 1][2]
      h = sortedObj[len(objects) - 1][3]
      # print(x,y,w,h)
      cv2.rectangle(out3, (int(x), int(y)), (int(x+w), int(y+h)), (0,255,0), 2)

   if not _live:
      cv2.imshow("field",out3)
   return (x,y,w,h)

def _ball(image, color, isFirst):

   if not _live:
      cv2.namedWindow("ball")
   out1 = _color_mask(image, 'ball',color,isFirst)

   #remove noise
   kernel = np.ones((2,2),np.uint8) #sets size of holes accepted i think?
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
      if radius > 2.0 and radius < 3:
         objects.append((pt[0],pt[1],radius))
  
   x=None
   y=None
   if len(objects) >= 1:
      sortedObj=sorted(objects,key=lambda x: x[2])
      x=sortedObj[0][0]
      y=sortedObj[0][1]
      # print('ball',x,y,sortedObj[0][2])
      cv2.circle(out3, (int(x), int(y)), int(sortedObj[0][2]), (100,100,255), -1)
   
   if not _live:
      cv2.imshow("ball",out3)
   return (x,y,0)

def main():
   rospy.init_node('vision', anonymous=False)

   # subscribe to camera
   rospy.Subscriber('camera', Image, _process_img)

   global us1_pub, us2_pub, ball_pub
   # publish locations
   us1_pub   = rospy.Publisher('vision/us1', Pose2D, queue_size=10)
   us2_pub   = rospy.Publisher('vision/us2', Pose2D, queue_size=10)
   # them1_pub = rospy.Publisher('vision/them1', Pose2D, queue_size=10)
   # them2_pub = rospy.Publisher('vision/them2', Pose2D, queue_size=10)
   ball_pub  = rospy.Publisher('vision/ball', Pose2D, queue_size=10)

   rospy.spin()
   rate = rospy.Rate(100) # 100 Hz
   while not rospy.is_shutdown():


      # Wait however long it takes to make this tick at 100Hz
      rate.sleep()

if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()
