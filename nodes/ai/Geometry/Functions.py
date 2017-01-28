#!/usr/bin/env python
"""Functions.py

This file contains common functions for doing geometric calculations, etc.

"""

from Models import Point, Vector, Line, Angle
# from .Models import Point, Vector, Line, Angle
import math


def dist(p1, p2):
   """Compute the distance between two points.

   p1 (Point)     : point 1
   p2 (Point)     : point 2
   return (Float) : distance between point 1 and point 2

   """
   return math.sqrt((p2.y - p1.y)**2 + (p2.x - p1.x)**2)

def dist_to_line(p, line):
   """Compute the distance from a point to a line.

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


def get_vector_between_points(p1, p2):
   """Returns a vector between two points.

   p1 (Point)      : first point
   p2 (Point)      : second point
   return (Vector) : vector between points

   """
   return Vector(p2.x-p1.x, p2.y-p1.y)


def get_vector_magnitude(v):
   """Get the magnitude of a vector.

   v (Vector)     : input vector
   return (Float) : magnitude of v

   """
   return math.sqrt(v.x**2 + v.y**2)


def normalize_vector(v):
   """Return a vector with same direction and magnitude 1.

   v (Vector)     : input vector
   return (Float) : vector with same direction and magnitude 1

   """
   m = get_vector_magnitude(v)
   return Vector(v.x/m, v.y/m)


def get_normalized_vector(angle):
   """Return a vector in the given angle with magnitude 1.

   angle (Angle)   : input angle
   return (Vector) : vector in given angle with magnitude 1

   """
   a = angle.radian
   return Vector(math.cos(a), math.sin(a))


def scale_vector(v, k):
   """Return a vector scaled by a constant factor.

   v (Vector)     : input vector
   k (Number)     : scale factor
   return (Float) : vector v scaled by constant k

   """
   return Vector(v.x*k, v.y*k)


def get_angle_of_vector(v):
   """Return the angle in radians of a vector.

   v (Vector)     : input vector
   return (Angle) : angle in radians between 0 and 2*pi

   """
   a = math.atan(v.y/v.x)
   if v.x < 0:
      a += math.pi
   elif v.y < 0:
      a += 2*math.pi
   return Angle(a, False)


def line2vec(line):
   """Return a vector from the beginning of the line to the end.

   line (Line)     : input line
   return (Vector) : vector representing going from the begninning of the line
                     to the end of the line

   """
   return Vector(line.end.x - line.beg.x, line.end.y - line.beg.y)


def closest_point_on_ellipse(center, x_radius, y_radius, point):
   if x_radius >= y_radius:
      e0, e1 = x_radius, y_radius
      c0, c1 = center.x, center.y
      y0, y1 = point.x - c0, point.y - c1
      swap = False
   else:
      e0, e1 = y_radius, x_radius
      c0, c1 = center.y, center.x
      y0, y1 = point.y - c0, point.x - c1
      swap = True

   if y0 < 0:
      if y1 < y0:
         p = _closest_point_on_norm_ellipse(e0, e1, -y0, -y1)
         p.x, p.y = -p.x, -p.y
         pass
      else:
         p = _closest_point_on_norm_ellipse(e0, e1, -y0, y1)
         p.x = p.x
         # p = closest_point_on_ellipse(center, x_radius, y_radius,
         #                              Point(y0, -y1))
         # return Point(p.x, -p.y)
   else:
      if y1 < y0:
         p = _closest_point_on_norm_ellipse(e0, e1, y0, -y1)
         p.y = -p.y
         # p = closest_point_on_ellipse(center_point, x_radius, y_radius,
         #                              Point(-y0, y1))
      else:
         p = _closest_point_on_norm_ellipse(e0, e1, y0, y1)

   if swap:
      return Point(p.y, p.x)
   else:
      return p


def _closest_point_on_norm_ellipse(e0, e1, y0, y1):
   # https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
   # e0 >= e1 > 0
   assert e0 >= e1 and e1 > 0
   assert y0 >= 0 and y1 >= 0
   # if x_radius >= y_radius:
   #    e0, e1 = x_radius, y_radius
   #    c0, c1 = center.x, center.y
   #    y0, y1 = point.x - c0, point.y - c1
   #    swap = False
   # else:
   #    e0, e1 = y_radius, x_radius
   #    c0, c1 = center.y, center.x
   #    y0, y1 = point.y - c0, point.x - c1
   #    swap = True

   # if y0 < 0:
   #    p = closest_point_on_ellipse(center, x_radius, y_radius,
   #                                 Point(y0, -y1))
   #    return Point(p.x, -p.y)
   # if y1 < 0:
   #    p = closest_point_on_ellipse(center_point, x_radius, y_radius,
   #                                 Point(-y0, y1))
   #    return Point(-p.x, p.y)

   # y0 >= 0 and y1 >= 0
   if y1 > 0:
      if y0 > 0:
         tbar = 1 # implement this
         x0 = e0**2*y0/(tbar + e0**2)
         x1 = e1**2*y1/(tbar + e1**2)
      else:
         x0 = 0
         x1 = e1
   else: # y1 == 0
      if y0 < ((e0**2 - e1**2)/e0):
         x0 = e0**2*y0/(e0**2 - e1**2)
         x1 = e1*math.sqrt(1 - (x0/e0)**2)
      else:
         x0 = e0
         x1 = 0

   return Point(x0, x1)
   # if swap:
   #    return Point(x1 + c1, x0 + c0)
   # else:
   #    return Point(x0 + c0, x1 + c1)



# Testing
def test():
   def is_close(v1, v2):
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
       if not is_close(r, e)]

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
          'Get_vector_between_points(p4,p1)']
   disp_errors(rec,exp,mes)


   # ellipse
   def print_ell(x,y,p,c,e):
      print 'x_axis: {}'.format(x)
      print 'y_axis: {}'.format(y)
      print 'point:  {}'.format(p)
      print 'center: {}'.format(c)
      print 'ell_pt: {}'.format(e)
      print 'dist:   {}'.format(get_vector_magnitude(e - c))
   x_axis = 0.22
   y_axis = 0.309

   point  = Point(-1.2, 0.4)
   center = Point(-1.7, 0.1)
   ell_pt = closest_point_on_ellipse(center, x_axis, y_axis, point)
   print_ell(x_axis, y_axis, point, center, ell_pt)

   point  = Point(1.2, 0.4)
   ell_pt = closest_point_on_ellipse(center, x_axis, y_axis, point)
   print_ell(x_axis, y_axis, point, center, ell_pt)

   point  = Point(1.2, -0.4)
   ell_pt = closest_point_on_ellipse(center, x_axis, y_axis, point)
   print_ell(x_axis, y_axis, point, center, ell_pt)

   point  = Point(-1.2, -0.4)
   ell_pt = closest_point_on_ellipse(center, x_axis, y_axis, point)
   print_ell(x_axis, y_axis, point, center, ell_pt)

   print('All tests passed')


if __name__ == '__main__':
   test()
