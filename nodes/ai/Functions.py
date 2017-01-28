"""Helpful functions to get information about game"""

from Geometry.Models import Position, Point, Line, Angle
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
   dir_vec    = gf.get_normalized_vector(angle)
   offset_vec = gf.scale_vector(dir_vec, -distance)
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
   return gf.get_angle_of_vector(Vector(
      game_state.game_info.get_home_goal_point(),
      game_state.field.ball.point))


# def get_cur_home_goal_point(game_info):
#    if game_info.side == Constants.left_side:
#       return Point(Constants.goal_point_left_x,
#                    Constants.goal_point_left_y)
#    elif game_info.side == Constants.right_side:
#       return Point(Constants.goal_point_right_x,
#                    Constants.goal_point_right_y)
#    raise Exception('invalid side in GameInfo')

# def get_cur_opp_goal_point(game_info):
#    if game_info.side == Constants.left_side:
#       return Point(Constants.goal_point_right_x,
#                    Constants.goal_point_right_y)
#    elif game_info.side == Constants.right_side:
#       return Point(Constants.goal_point_left_x,
#                    Constants.goal_point_left_y)
#    raise Exception('invalid side in GameInfo')


def test():
   from Models import Field, Angle, GameInfo, GameState
   import Constants
   f = Field()
   print f
   a = Angle(33, True)
   d1 = 0.2
   p1 = get_point_behind_ball(f, a, d1)
   print 'get_point_behind_ball(f, {}, {}) = {}'.format(a, d1, p1)
   print 'distance = {}'.format(gf.dist(f.ball.point, p1))
   d2 = 3.0
   p2 = get_point_behind_ball(f, a, d2)
   print 'get_point_behind_ball(f, {}, {}) = {}'.format(a, d2, p2)
   print 'distance = {}'.format(gf.dist(f.ball.point, p2))
   gi1 = GameInfo(Constants.left_side)
   gs1 = GameState(f, gi1)
   l1  = get_line_goal2ball(gs1)
   v1  = gf.line2vec(l1)
   print 'line1: {} ({}, {} long'.format(l1, v1,
                                          gf.get_vector_magnitude(v1))
   gi2 = GameInfo(Constants.right_side)
   gs2 = GameState(f, gi2)
   l2  = get_line_goal2ball(gs2)
   v2  = gf.line2vec(l2)
   print 'line2: {} ({}, {} long'.format(l2, v2,
                                          gf.get_vector_magnitude(v2))



if __name__ == '__main__':
   test()
