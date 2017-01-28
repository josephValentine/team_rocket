"""Custom objects for geometrical constructs."""

import math

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
