"""Elementary actions"""

import Constants
from Geometry.Models import Point

# Elementary
def move_to_point_abs(robot, point):
   set_commanded_point(point)
   pass

def move_to_point_rel(robot, point):
   set_commanded_point((robot.x + point.x, robot.y + point.y))
   pass

def rotate_to_ang_abs(robot, ang):
   pass

def rotate_by_ang_rel(robot, ang):
   pass

def move_to_position(robot, position):
   set_commanded_pos(robot, (position.point.x, position.point.y))
   set_commanded_ang(robot, position.ang)

def kick(robot):
   # call kicker
   pass
