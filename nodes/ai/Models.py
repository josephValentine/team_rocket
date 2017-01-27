"""Models for robots, ball, etc."""

import Constants
import math

class Robot:
    """Holds information about a robot."""
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
    """Holds information about the ball."""
    def __init__(self, point):
        self.point = point
    def __eq__(self, other):
        return type(self) == type(other)
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Ball({})'.format(self.point)


class Point:
    """An x,y cartesian coordinate pair."""
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
    def __add__(self, other):
        """Adds a vector to a point.

        Adds the x, y values of the vector to the point. All other types
        (including another Point) are not defined. It does not make sense to add
        a point to another point.

        """
        if type(other) != Vector:
            raise TypeError(_type_error_str(self, other))
        return Point(self.x + other.x, self.y + other.y)
    def __sub__(self, other):
        """Subtracts a vector or another vector from a point.

        Subtracts the x, y values of the Point or Vector from the point. All
        other types are not defined. It does not make sense to add a point to
        another point.

        """
        if type(other) != Point and type(other) != Vector:
            raise TypeError(_type_error_str(self, other))
        return Vector(self.x - other.x, self.y - other.y)


class Vector:
    """An x,y cartesian coordinate vector."""
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
    def __add__(self, other):
        if type(other) != Vector and type(other) != Point:
            raise TypeError(_type_error_str(self, other))
        return other.__class__(self.x + other.x, self.y + other.y)
    def __sub__(self, other):
        if type(other) != Vector:
            raise TypeError(_type_error_str(self, other))
        return Vector(self.x - other.x, self.y - other.y)


class Line:
    """A pair of lines that define a point.

    The beginning and end points may or may not constitude the end points of a
    line segment, depending on the usage.

    """
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
    """An angle, stored in both radians and degrees."""
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
        return 'Angle({}, {})'.format(self.degree, True)
    def __add__(self, other):
        if type(other) == Angle:
            return Angle(self.radian + other.radian, False)
        raise TypeError(_type_error_str(self, other))
    def __sub__(self, other):
        if type(other) == Angle:
            return Angle(self.radian - other.radian, False)
        raise TypeError(_type_error_str(self, other))


class Position:
    """Information about a position, including a point and angle."""
    def __init__(self, point, angle):
        self.point = point
        self.angle = angle
    def __add__(self, other):
        if type(other) == Vector:
            return Position(self.point + other, self.angle)
        if type(other) == Angle:
            return Position(self.point, self.angle + other)
        raise TypeError(_type_error_str(self, other))
    def __sub__(self, other):
        if type(other) == Vector:
            return Position(self.point - other, self.angle)
        if type(other) == Angle:
            return Position(self.point, self.angle - other)
        raise TypeError(_type_error_str(self, other))
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Position({}, {})'.format(self.point, self.angle)



class Field:
    """Holds information about everything on the field.

    This includes all robots and the ball.

    """
    def __init__(self):
        self.ball  = Ball(Point(Constants.ball_start_x,
                                Constants.ball_start_y))
        self.ally1 = Robot(Constants.team_us,
                           Constants.ally_1_id,
                           Point(Constants.ally_1_start_x,
                                 Constants.ally_1_start_y))
        self.ally2 = Robot(Constants.team_us,
                           Constants.ally_2_id,
                           Point(Constants.ally_2_start_x,
                                 Constants.ally_2_start_y))
        self.opp1  = Robot(Constants.team_them,
                           Constants.opp_1_id,
                           Point(Constants.opp_1_start_x,
                                 Constants.opp_1_start_y))
        self.opp2  = Robot(Constants.team_them,
                           Constants.opp_2_id,
                           Point(Constants.opp_2_start_x,
                                 Constants.opp_2_start_y))
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Field({}, {}, {}, {}, {})'.format(
            self.ball, self.ally1, self.ally2, self.opp1, self.opp2)


class GameHistory:
    """Information about what has happened in the game so far.

    This could contain strategies for our team or our opponenets, how aggressive
    each team was, etc.

    """
    def __init__(self):
        pass
    def __str__(self):
        return 'Unimplemented GameHistory'
    def __repr__(self):
        return 'GameHistory()'


class GameState:
    """Information about the past and current state of the game

    This includes the current field and the past history.

    """
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


def _type_str(x):
    return str(type(x))[8:-2]


def _type_error_str(x, y):
    return 'unsupported operand type(s) for: \''+_type_str(x)+\
        '\' and \''+_type_str(y)+'\''
