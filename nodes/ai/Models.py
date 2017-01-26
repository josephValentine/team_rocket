""" Models for robots, ball, etc. """

class Robot:
    def __init__(self, team, id, pos):
       self.team = team
       self.id   = id
       self.pos  = pos
    def __str__(self):
        return 'Robot({}, {}, {})'.format(self.team, self.id, self.pos)


class Ball:
    def __init__(self, pos):
        self.pos = pos
    def __str__(self):
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
        return 'Vector({}, {})'.format(self.x, self.y)


class Line:
    def __init__(self, beg, end):
        self.beg = beg
        self.end = end
    def __eq__(self, other):
        return type(other) == type(self) and \
            self.beg == other.beg and \
            self.end == other.end
    def __str__(self):
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
        return 'Point({}, {})'.format(self.degree, True)


def _rad_to_deg(rad):
   return rad*180/math.pi


def _deg_to_rad(deg):
   return deg*math.pi/180
