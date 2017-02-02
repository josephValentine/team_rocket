"""Helpful functions to get information about game"""

from Geometry.Models import Position, Point, Line, Angle, Vector
import Geometry.Functions as gf
import Constants

def get_point_behind_ball(field, angle, distance):
   """Get the point behind the ball facing a given direction.

   field (Field)    : Current state of game field
   angle (Angle)    : Direction to face
   distance (Float) : The distance the point should be behind the ball
   return (Point)   : A point behind the ball

   """
   ball_point = field.ball.point
   offset_vec = angle.get_normalized_vector().get_scaled(-distance)
   return ball_point + offset_vec


def get_line_goal2ball(game_state):
   """Get a line from the center of the goal to the ball

   game_state (GameState) : Current state of game
   return (Line)          : A line with beginning at goal and end at ball

   """
   return Line(game_state.game_info.get_home_goal_point(),
               game_state.field.ball.point)


def get_angle_goal2ball(game_state):
   """Get the angle from the center of the goal to the ball

   game_state (GameState) : Current state of game
   return (Angle)         : Angle from center of goal to the ball

   """
   return (game_state.field.ball.point -
           game_state.game_info.get_home_goal_point()).get_angle()


def test():
   from Models import Field, Angle, GameInfo, GameState
   import Constants
   f = Field()
   print f
   a = Angle(33, True)
   d1 = 0.2
   p1 = get_point_behind_ball(f, a, d1)
   print 'get_point_behind_ball(f, {}, {}) = {}'.format(a, d1, p1)
   print 'distance = {}'.format(f.ball.point.dist_to_point(p1))
   d2 = 3.0
   p2 = get_point_behind_ball(f, a, d2)
   print 'get_point_behind_ball(f, {}, {}) = {}'.format(a, d2, p2)
   print 'distance = {}'.format(f.ball.point.dist_to_point(p2))
   gi1 = GameInfo(Constants.left_side)
   gs1 = GameState(f, gi1)
   l1  = get_line_goal2ball(gs1)
   v1  = l1.to_vector()
   print 'line1: {} ({}, {} long)'.format(l1, v1, v1.get_magnitude())
   a1 = get_angle_goal2ball(gs1)
   print 'angle1: {}'.format(a1)
   gi2 = GameInfo(Constants.right_side)
   gs2 = GameState(f, gi2)
   l2  = get_line_goal2ball(gs2)
   v2  = l2.to_vector()
   print 'line2: {} ({}, {} long)'.format(l2, v2, v2.get_magnitude())


if __name__ == '__main__':
   test()
