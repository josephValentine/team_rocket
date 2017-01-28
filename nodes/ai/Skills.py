"""Combination of Utilities to perform simple tasks"""

import Functions
import Geometry.Functions as gf
import Constants

def stay_between_goalnball(game_state, robot):
   goalie_position = get_goalie_position(game_state,
                                         Constants.goalie_dist_from_goal)
   if True: # Check if it's a good idea
      return goalie_position.to_tuple()
   else:
      return robot.position.to_tuple()


def get_goalie_position(game_state, distance):
   """Return a point at specified distance from goal toward the ball

   game_state (GameState) : Current state of game
   distance (Float)       : Distance from goal
   return (Point)         : A point `distance` away from goal in direction of
                            ball

   """
   goal_point = game_state.game_info.get_home_goal_point()
   ball_point = game_state.field.ball.point
   vec        = ball_point - goal_point
   angle      = gf.get_angle_of_vector(vec)
   offset     = gf.scale_vector(gf.get_normalized_vector(angle), distance)
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


