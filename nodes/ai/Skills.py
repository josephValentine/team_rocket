"""Combination of Utilities to perform simple tasks"""

import Functions
from Geometry.Models import Position, Point
import Geometry.Functions as gf
import Constants

def stay_between_goalnball(game_state, robot):
   # goalie_position = get_goalie_position(game_state,
   #                                       Constants.goalie_dist_from_goal)
   goalie_position = get_smart_goalie_position(game_state,
                                               Constants.goalie_dist_from_goal)
   if True: # Check if it's a good idea
      return goalie_position
   else:
      return robot.position


def push_ball_in_direction(game_state, robot, angle):
   """Get behind ball and start pushing in direction.

   Get behind the ball relative to given angle, then start pushing the ball in
   the given angle.

   Assumptions:
   - There is nobody between us and the ball.
   - We are not directly in front of the ball.

   """
   pass


def get_goalie_position(game_state, distance):
   """Return a point at specified distance from goal toward the ball

   game_state (GameState) : Current state of game
   distance (Float)       : Distance from goal
   return (Point)         : A point `distance` away from goal in direction of
                            ball

   """
   ball_point = game_state.field.ball.point
   # #### Ellipse ####
   # ell_point  = get_ellipse_position(game_state)
   # vec = ball_point - ell_point
   # angle = gf.get_angle_of_vector(vec)
   # return Position(ell_point, angle)
   # #################
   goal_point = game_state.game_info.get_home_goal_point()
   vec        = ball_point - goal_point
   angle      = gf.get_angle_of_vector(vec)
   offset     = gf.scale_vector(gf.get_normalized_vector(angle), distance)
   return Position(goal_point + offset, angle)


# This isn't working yet
def get_ellipse_position(game_state):
   x_axis = Constants.goalie_dist_from_goal
   y_axis = Constants.goal_box_width/2
   point  = game_state.field.ball.point
   center = Vector(Constants.goal_point_left_x, Constants.goal_point_left_y)
   norm_point = point - center
   swapped_point = Point(norm_point.y, norm_point.x) # because x < y for ellipse
   return gf.closest_point_on_ellipse(center, x_axis, y_axis, point)


count = 0
prev_ball_point = Point(0, 0)
prev_prev_ball_point = Point(0, 0)
field_width = 3.40
def get_smart_goalie_position(game_state, distance):
   """Return a point at specified distance from goal toward the ball

   It will be the closest point from the ball to goal line, where the goal ends
   on top and bottom.

   game_state (GameState) : Current state of game
   distance (Float)       : Distance from goal
   return (Point)         : A point `distance` away from goal in direction of
                            ball

   """
   field_half_w = field_width/2
   if self.game_state.game_info.side == 'away':
      field_half_w = -field_half_w

   global count, prev_ball_point, prev_prev_ball_point
   ball_point = game_state.field.ball.point
   count = (count + 1) % 5
   if count == 0:
      prev_prev_ball_point = prev_ball_point
      prev_ball_point = ball_point
   delta_x = prev_ball_point.x - prev_prev_ball_point.x
   if count == 0:
      print 'delta_x = {}'.format(delta_x)
   goal_center_point = field_half_w # game_state.game_info.get_home_goal_point()
   if abs(delta_x) > 0.01:
      delta_y = prev_ball_point.y - prev_prev_ball_point.y
      dist_to_goal = goal_center_point - prev_prev_ball_point.x
      future_y = delta_y / delta_x * dist_to_goal
   else:
      future_y = ball_point.y
   if future_y > Constants.goal_top_y:
      goal_point = Point(goal_center_point, Constants.goal_top_y)
   elif future_y < Constants.goal_bottom_y:
      goal_point = Point(goal_center_point, Constants.goal_bottom_y)
   else:
      goal_point = Point(goal_center_point, future_y)
   vec    = ball_point - goal_point
   angle  = vec.get_angle()
   # print "commanded angle: %s" % angle
   offset = angle.get_normalized_vector().get_scaled(distance)
   return Position(goal_point + offset, angle)


# def kick_ball_toward_pos(robot, ball, pos):
#    ang = find_angle(ball, pos)
#    rotate_to_ang_abs(robot, ang)
#    if ready_to_kick(robot, ball):
#       kick(robot)


# def ready_to_kick(robot, ball):
#    return ball_close_to_kicker and not other_robot_close_to_kicker(robot)


# def ball_close_to_kicker(robot, ball):
#    distance = find_distance(robot, ball)
#    return distance <= min_kick_distance



# def push_ball_toward_pos(robot, ball, pos):



def test():
   from Models import GameState, Field, GameInfo

   f1  = Field()
   gi1 = GameInfo(Constants.left_side)
   gs1 = GameState(f1, gi1)
   cmds1 = stay_between_goalnball(gs1, f1.ally2)
   print 'cmds1: {}'.format(cmds1)

   f2  = Field()
   f2.ball.point = Point(1.5, -1.0)
   gi2 = GameInfo(Constants.right_side)
   gs2 = GameState(f2, gi2)
   cmds2 = stay_between_goalnball(gs2, f2.ally2)
   print 'cmds2: {}'.format(cmds2)


if __name__ == '__main__':
   test()
