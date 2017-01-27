""" Models for robots, ball, etc. """

import Constants

class Robot:
    def __init__(self, team, id, pos):
       self.team = team
       self.id   = id
       self.pos  = pos
    def __eq__(self, other):
        return type(self) == type(other) and self.id == other.id
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Robot({}, {}, {})'.format(self.team, self.id, self.pos)


class Ball:
    def __init__(self, pos):
        self.pos = pos
    def __eq__(self, other):
        return type(self) == type(other)
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Ball({})'.format(self.pos)


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return type(other) == type(self) and \
            self.x == other.x and \
            self.y == other.y
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Point({}, {})'.format(self.x, self.y)


# Do we want a seperate class? What would be different?
class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return type(other) == type(self) and \
            self.x == other.x and \
            self.y == other.y
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Vector({}, {})'.format(self.x, self.y)
    # def __add__(self):
    #     pass
    # def __sub__(self):
    #     pass


class Line:
    def __init__(self, beg, end):
        self.beg = beg
        self.end = end
    def __eq__(self, other):
        return type(other) == type(self) and \
            self.beg == other.beg and \
            self.end == other.end
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Line({}, {})'.format(self.beg, self.end)


class Angle:
    def __init__(self, value, is_degree):
        if is_degree:
            self.degree = value
            self.radian = _deg_to_rad(value)
        else:
            self.degree = _rad_to_deg(value)
            self.radian = value
    def __eq__(self, other):
        return type(other) == type(self) and \
            self.degree == other.degree and \
            self.radian == other.radian
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Point({}, {})'.format(self.degree, True)


class Field:
    def __init__(self):
        self.ball  = Ball(Point(Constants.ball_start_x,
                                Constants.ball_start_y))
        self.ally1 = Robot(Point(Constants.ally_1_start_x,
                                 Constants.ally_1_start_y))
        self.ally2 = Robot(Point(Constants.ally_2_start_x,
                                 Constants.ally_2_start_y))
        self.opp1  = Robot(Point(Constants.opp_1_start_x,
                                 Constants.opp_1_start_y))
        self.opp2  = Robot(Point(Constants.opp_2_start_x,
                                 Constants.opp_2_start_y))
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Board({}, {}, {}, {}, {})'.format(
            self.ball, self.ally1, self.ally2, self.opp1, self.opp2)


class GameHistory:
    def __init__(self):
        pass
    def __str__(self):
        return 'Uninitialized GameHistory'
    def __repr__(self):
        return 'GameHistory()'


class GameState:
    def __init__(self, field, game_history):
        self.field        = field
        self.game_history = game_history
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'GameState({}, {})'.format(self.field, self.game_history)



# Helper functions
def _rad_to_deg(rad):
   return rad*180/math.pi


def _deg_to_rad(deg):
   return deg*math.pi/180
