"""Custom objects for geometrical constructs."""

import math

class Position(object):
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
    def update_x(self, x):
        self.point.update_x(x)
    def update_y(self, y):
        self.point.update_y(y)
    def update_angle(self, angle, is_degree):
        self.angle.update(angle, is_degree)
    def to_tuple(self):
        return (self.point.x, self.point.y, self.angle.radian)
    def from_tuple(tup):
        return Position(Point(tup[0], tup[1]), Angle(tup[2], False))


class Point(object):
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
    def to_tuple(self):
        return (self.x, self.y, 0)
    def from_tuple(self, tup):
        self.x, self.y = tup[0:2]
    def update_x(self, x):
        self.x = x
    def update_y(self, y):
        self.y = y
    def dist_to_line(self, line)
        """Return distance to a line.

        line (Line)   : line to calculate distance to
        return (Flat) : distance from self to line

        """
        return _dist_point_to_line(self, line)
    def dist_to_point(self, point):
        """Compute the distance between two points.

        point (Point)  : point to calculate distance to
        return (Float) : distance between self and point

        """
        return math.sqrt((p2.y - point.y)**2 + (p2.x - point.x)**2)



class Vector(object):
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
    def get_magnitude(self):
        """Return magnitude of vector."""
        return math.sqrt(self.x**2 + self.y**2)
    def get_normalized(self):
        """Return a normalized version of the vector."""
        m = self.get_magnitude()
        return Vector(self.x/m, self.y/m)
    def get_angle(self):
        """Return the angle the vector makes from x-axis."""
        a = math.atan(v.y/v.x)
        if v.x < 0:
            a += math.pi
        elif v.y < 0:
            a += 2*math.pi
        return Angle(a, False)


class Line(object):
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
    def to_vector(self):
        """Return Vector version of line from beginning to end."""
        return self.end - self.beg
    def dist_to_point(self, point):
        """Return distance to a point.

        point (Point)  : point to calculate distance to
        return (Float) : distance from self to point

        """
        return _dist_point_to_line(point, self)
    def intersection_with_line(self, line):
        """Return the point of intersection with another line.

        line (Line)  : line to find intersction with
        return (Flat) : point of intersection

        Throws NoIntersectionError if the lines do not intersect (are
        parrallel).

        """
        # https://en.wikipedia.org/wiki/
        # Line%E2%80%93line_intersection#Given_two_points_on_each_line
        x1, y1 = self.beg.x, self.beg.y
        x2, y2 = self.end.x, self.end.y
        x3, y3 = line.beg.x, line.beg.y
        x4, y4 = line.end.x, line.end.y
        x1y2, y1x2 = x1*y2, y1*x2
        x3y4, y3x4 = x3*y4, y3*x4
        x1_x2, x3_x4 = x1-x2, x3-x4
        y1_y2, y3_y4 = y1-y2, y3-y4
        a, b = x1y2-y1x2, x3y4-y3x4
        den = x1_x2*y3_y4 - y1_y2*x3_x4
        P_x = (a*x3_x4 - x1_x2*b)/den
        P_y = (a*y3_y4 - y1_y2*b)/den
        return Point(P_x, P_y)


class Angle(object):
    """An angle, stored in both radians and degrees."""
    def __init__(self, value, is_degree):
        self.update_angle(value, is_degree)
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
            return Angle(self.normalize(self.radian+other.radian, False), False)
        raise TypeError(_type_error_str(self, other))
    def __sub__(self, other):
        if type(other) == Angle:
            return Angle(self.normalize(self.radian-other.radian, False), False)
        raise TypeError(_type_error_str(self, other))
    def normalize(self, value, is_degree):
        max_value = 360 if is_degree else 2*math.pi
        return value % max_value
    def update_angle(self, value, is_degree):
        if is_degree:
            self.degree = value % 360
            self.radian = _deg_to_rad(self.degree)
        else:
            self.radian = value % (2*math.pi)
            self.degree = _rad_to_deg(self.radian)


# Helper functions
def _rad_to_deg(rad):
    return rad*180/math.pi


def _deg_to_rad(deg):
    return deg*math.pi/180


def _type_str(x):
    # return str(type(x))[8:-2]
    return str(type(x))


def _type_error_str(x, y):
    return 'unsupported operand type(s) for: \''+_type_str(x)+\
        '\' and \''+_type_str(y)+'\''

def _dist_point_to_line(p, line):
    p0 = p
    p1 = self.beg
    p2 = self.end
    num = abs((p2.y-p1.y)*p0.x - (p2.x-p1.x)*p0.y + p2.x*p1.y - p2.y*p1.x)
    den = math.sqrt((p2.y-p1.y)**2 + (p2.x-p1.x)**2)
    return num/den
