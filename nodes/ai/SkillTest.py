from Geometry.Models import Angle, Position, Point

class SkillTest(object):
   def __init__(self):
      super(SkillTest, self).__init__()
      self.game_state = GameState(Field(), GameInfo(team_side))
      self.timer = 0
   def update(self, me, ball, game_state):
      # print me, ally, opp1, opp2, ball, game_state, type(game_state)
      f = self.game_state.field
      f.ally1.position = _pose2d_to_position(me)
      f.ball.point = _pose2d_to_point(ball)
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
      # set desired angle to 5 degrees off from where we are
      dtheta = Angle(5, True)
      my_pos = f.ally1.position
      return Position(my_pos.point, my_pos.angle + dtheta)
   def go_to_center(self):
      # set desired position to be center facing goal
      center_point = Point(0,0)
      goal_angle = Angle(0, True)
      return Position(center_point, goal_angle)
   def move_in_box(self):
      if self.timer <= 1000:
         return Position(Point(1,0), Angle(0, True))
      raise NotImplemented('move_in_box')
