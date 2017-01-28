"""Elementary actions"""

import Constants
from Geometry import Point

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


# Helper
def get_cur_home_goal(game_info):
   if game_info.side == Constants.left_side:
      return Point(Constants.goal_point_left_x,
                   Constants.goal_point_left_y)
   elif game_info.side == Constants.right_side:
      return Point(Constants.goal_point_right_x,
                   Constants.goal_point_right_y)
   raise Exception('invalid side in GameInfo')

def get_cur_opp_goal(game_info):
   if game_info.side == Constants.left_side:
      return Point(Constants.goal_point_right_x,
                   Constants.goal_point_right_y)
   elif game_info.side == Constants.right_side:
      return Point(Constants.goal_point_left_x,
                   Constants.goal_point_left_y)
   raise Exception('invalid side in GameInfo')
