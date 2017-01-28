"""Elementary actions"""

import Constants
from Models import Point

# Elementary
def move_to_pos_abs(robot, pos):
   set_commanded_pos(pos)
   pass

def move_to_pos_rel(robot, pos):
   set_commanded_pos((robot.x + pos.x, robot.y + pos.y))
   pass

def rotate_to_ang_abs(robot, ang):
   pass

def rotate_by_ang_rel(robot, ang):
   pass

def move_to_pose2d(robot, pose2d):
   set_commanded_pos(robot, (pose2d.x, pose2d.y))
   set_commanded_ang(robot, pose2d.ang)

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
