from Geometry.Models import Angle, Position, Point
# from Models import GameState, Field, GameInfo
from geometry_msgs.msg import Pose2D

class SkillTest(object):
   def __init__(self):
      super(SkillTest, self).__init__()
      self.timer = 0
      self.me = Pose2D()
      self.ball = Pose2D()
   def update(self, me, ball):
      # print me, ally, opp1, opp2, ball, game_state, type(game_state)
      # f = self.game_state.field
      # f.ally1.position = _pose2d_to_position(me)
      # f.ball.point = _pose2d_to_point(ball)
      self.me = me
      self.ball = ball
      self.timer += 1
   def get_commanded_position(self, skill):
      if skill == 'spin':
         return self.spin()
      elif skill == 'go_to_center':
         return self.go_to_center()
      elif skill == 'move_in_box':
         return self.move_in_box()
      else:
         return self.spin()
   def spin(self):
      # f = self.game_state.field
      # set desired angle to 5 degrees off from where we are
      dtheta = Angle(30, True)
      print 'self.angle: {}, commanded angle: {}'.format(
         self.me.angle, self.me.angle + dtheta)
      return Position(self.me.point, self.me.angle + dtheta)
   def go_to_center(self):
      # set desired position to be center facing goal
      center_point = Point(0,0)
      goal_angle = Angle(0, True)
      return Position(center_point, goal_angle)
   def move_in_box(self):
      if self.timer <= 1000:
         return Position(Point(1,0), Angle(0, True))
      raise NotImplemented('move_in_box')

def _pose2d_to_position(pose2d):
   return Position(Point(pose2d.x, pose2d.y), Angle(pose2d.theta, True))

def _pose2d_to_point(pose2d):
    return Point(pose2d.x, pose2d.y)
