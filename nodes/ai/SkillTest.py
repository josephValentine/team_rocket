from Geometry.Models import Angle, Position, Point, Vector
# from Models import GameState, Field, GameInfo
from geometry_msgs.msg import Pose2D

box_dests = [(0.05,0), (0,0), (0,0.05), (0,0), (-0.05,0), (0,0), (0,-0.05)]
box_dests_times = [(0.05,0,100), (0,0,200), (0,0.05,100), (0,0,200),
                   (-0.05,0,100), (0,0,200), (0,-0.05,100), (0,0,200)]
box_i = 0
max_timer = 50

class SkillTest(object):
   def __init__(self):
      super(SkillTest, self).__init__()
      self.timer = 0
      self.me   = Position(Point(0,0), Angle(0,True))
      self.ball = Position(Point(0,0), Angle(0,True))
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
      # print 'self.angle: {}, commanded angle: {}'.format(
      #    self.me.angle, self.me.angle + dtheta)
      return Position(self.me.point, self.me.angle + dtheta)
   def go_to_center(self):
      # set desired position to be center facing goal
      center_point = Point(0,0)
      goal_angle = Angle(0, True)
      return Position(center_point, goal_angle)
   def move_in_box(self):
      global box_dests, box_dests_times, box_i, max_timer
      # if (self.timer + 1) % 20 == 0:
      #    print 'timer: %d' % self.timer
      # if (self.timer % max_timer) == (max_timer - 1):
      if (self.timer % max_timer) == (max_timer - 1):
         box_i += 1
         self.timer = 0
         # if box_i < len(box_dests):
         if box_i < len(box_dests_times):
            print '-'*80
            print 'new box dest'
            print box_dests_times[box_i]
            print '-'*80
      # if box_i < len(box_dests):
      if box_i < len(box_dests_times):
         # return Position(self.me.point + Vector(*box_dests[box_i][:2]),
         #                 self.me.angle)
         max_timer = box_dests_times[box_i][2]
         return Position(self.me.point + Vector(*box_dests_times[box_i][:2]),
                         self.me.angle)
      else:
         return self.me
      # raise NotImplemented('move_in_box')

def _pose2d_to_position(pose2d):
   return Position(Point(pose2d.x, pose2d.y), Angle(pose2d.theta, True))

def _pose2d_to_point(pose2d):
    return Point(pose2d.x, pose2d.y)

delta = 0.05
def _close(p1, p2):
   return p1.dist_to_point(p2) < delta
