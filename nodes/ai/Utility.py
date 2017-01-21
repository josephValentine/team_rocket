#!/usr/bin/env python

""" Utility.py

This file contains common functions for doing geometric calculations,
etc.

"""

from geometry_msgs.msg import Pose2D
import math


def dist(p1, p2):
   """ Compute the distance between two points

   p1 (Pose2D)    : point 1
   p2 (Pose2D)    : point 2
   return (Float) : distance between point 1 and point 2
   """
   print("math.sqrt(({} - {})**2 + ({} - {})**2)".
         format(p2.y,p1.y,p2.x,p1.x))
   return math.sqrt((p2.y - p1.y)**2 + (p2.x - p1.x)**2)

def dist_to_line(p, line):
   """ Compute the distance from a point to a line

   p (Pose2D)              : point to calculate distance from
   line ((Pose2D, Pose2D)) : tuple of two points that define a line
   return (Float)          : distance from p to line
   """
   p0 = p
   p1 = line[0]
   p2 = line[1]
   # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
   num = abs((p2.y-p1.y)*p0.x - (p2.x-p1.x)*p0.y + p2.x*p1.y - p2.y*p1.x)
   den = math.sqrt((p2.y-p1.y)**2 + (p2.x-p1.x)**2)
   return num/den
