#!/usr/bin/env python

""" Functions.py

This file contains common functions for doing geometric calculations,
etc.

"""

# from geometry_msgs.msg import Pose2D
from Models import Point, Vector, Line, Angle
import math


def dist(p1, p2):
   """ Compute the distance between two points

   p1 (Point)     : point 1
   p2 (Point)     : point 2
   return (Float) : distance between point 1 and point 2
   """
   return math.sqrt((p2.y - p1.y)**2 + (p2.x - p1.x)**2)

def dist_to_line(p, line):
   """ Compute the distance from a point to a line

   p (Point)      : point to calculate distance from
   line (Line)    : tuple of two points that define a line
   return (Float) : distance from p to line
   """
   p0 = p
   p1 = line.beg
   p2 = line.end
   # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
   num = abs((p2.y-p1.y)*p0.x - (p2.x-p1.x)*p0.y + p2.x*p1.y - p2.y*p1.x)
   den = math.sqrt((p2.y-p1.y)**2 + (p2.x-p1.x)**2)
   return num/den


def rad_to_deg(rad):
   """ Converts radians to degrees

   rad (Float)    : angle in radians
   return (Float) : angle in degrees
   """
   return rad*180/math.pi


def deg_to_rad(deg):
   """ Converts degrees to radians

   rad (Float)    : angle in degrees
   return (Float) : angle in radians
   """
   return deg*math.pi/180


def get_vector_between_points(p1, p2):
   """ Returns a vector between two points

   p1 (Point)      : first point
   p2 (Point)      : second point
   return (Vector) : vector between points
   """
   return Vector(p2.x-p1.x, p2.y-p1.y)


def get_vector_magnitude(v):
   """ Get the magnitude of a vector

   v (Vector)     : input vector
   return (Float) : magnitude of v
   """
   return math.sqrt(v.x**2 + v.y**2)


def normalize_vector(v):
   """ Return a vector with same direction and magnitude 1

   v (Vector)     : input vector
   return (Float) : vector with same direction and magnitude 1
   """
   m = get_vector_magnitude(v)
   return Vector(v.x/m, v.y/m)


def scale_vector(v, k):
   """ Return a vector scaled by a constant factor

   v (Vector)     : input vector
   k (Number)     : scale factor
   return (Float) : vector v scaled by constant k
   """
   return Vector(v.x*k, v.y*k)


def get_angle_of_vector(v):
   """ Return the angle in radians of a vector

   v (Vector)     : input vector
   return (Float) : angle in radians
   """
   return math.atan(v.y/v.x)



# Testing
def test():
   def close(v1, v2):
      delta = 0.0000001
      if v1 == v2: return True
      if type(v1) != type(v2): return False
      if type(v1) == int: return abs(v1-v2) < delta
      return False
   def disp_error(rec, exp, mes):
      print('Received {}, expected {} for {}'.format(rec, exp, mes))
      exit()
   def disp_errors(recs, exps, mess):
      # print('recs ({}): {}\nexps ({}): {}\nmess ({}): {}\n'.format(
      #       type(recs[0]), recs, type(exps[0]), exps, type(mess[0]), mess))
      [disp_error(r, e, m) for r, e, m in zip(rec, exp, mes) \
       if not close(r, e)]

   p0 = Point(0,0)
   p1 = Point(-1,-1)
   p2 = Point(-1, 1)
   p3 = Point( 1, 1)
   p4 = Point( 1,-1)
   l12 = Line(p1,p2)
   l23 = Line(p2,p3)
   l34 = Line(p3,p4)
   l41 = Line(p4,p1)

   rec = [  dist(p1,p2),   dist(p2,p3),   dist(p3,p4),   dist(p4,p1)]
   exp = [2]*len(rec)
   mes = ['dist(p1,p2)', 'dist(p2,p3)', 'dist(p3,p4)', 'dist(p4,p1)']
   disp_errors(rec,exp,mes)

   rec = [  dist(p2,p1),   dist(p3,p2),   dist(p4,p3),   dist(p1,p4)]
   exp = [2]*len(rec)
   mes = ['dist(p2,p1)', 'dist(p3,p2)', 'dist(p4,p3)', 'dist(p1,p4)']
   disp_errors(rec,exp,mes)

   rec = [  dist(p1,p3),   dist(p3,p1),   dist(p2,p4),   dist(p4,p2)]
   exp = [math.sqrt(8)]
   mes = ['dist(p1,p3)', 'dist(p3,p1)', 'dist(p2,p4)', 'dist(p4,p2)']
   disp_errors(rec,exp,mes)

   rec = [  dist_to_line(p0,l12),   dist_to_line(p0,l12),
            dist_to_line(p0,l12),   dist_to_line(p0,l12)]
   exp = [1]*len(rec)
   mes = ['dist_to_line(p0,l12)', 'dist_to_line(p0,l12)',
          'dist_to_line(p0,l12)', 'dist_to_line(p0,l12)']
   disp_errors(rec,exp,mes)

   rec = [  get_vector_between_points(p1,p2),
            get_vector_between_points(p2,p3),
            get_vector_between_points(p3,p4),
            get_vector_between_points(p4,p1)]
   exp = [Vector(0,2), Vector(2,0), Vector(0,-2), Vector(-2,0)]
   mes = ['get_vector_between_points(p1,p2)',
          'get_vector_between_points(p2,p3)',
          'get_vector_between_points(p3,p4)',
          'get_vector_between_points(p4,p1)']
   disp_errors(rec,exp,mes)



   print('All tests passed')


if __name__ == '__main__':
   test()
