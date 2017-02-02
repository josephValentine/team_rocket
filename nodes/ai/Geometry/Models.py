"""Custom objects for geometrical constructs."""

import math

class Position(object):
    """Information about a position, including a point and angle."""
    def __init__(self, point, angle):
        self.point = point
        self.angle = angle
    def __eq__(self, other):
        return type(self) == type(other) and \
            self.point == other.point and \
            self.angle == other.angle
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
        return type(self) == type(other) and \
            _close(self.x, other.x) and \
            _close(self.y, other.y)
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
    def dist_to_line(self, line):
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
        return math.sqrt((self.y - point.y)**2 + (self.x - point.x)**2)



class Vector(object):
    """An x,y cartesian coordinate vector."""
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return type(self) == type(other) and \
            _close(self.x, other.x) and \
            _close(self.y, other.y)
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
        a = math.atan(self.y/self.x)
        if self.x < 0:
            a += math.pi
        elif self.y < 0:
            a += 2*math.pi
        return Angle(a, False)
    def get_scaled(self, k):
        """Return a vector scaled by a constant factor.

        k (Number)     : scale factor
        return (Vector) : self scaled by constant k

        """
        return Vector(k*self.x, k*self.y)

class Line(object):
    """A pair of lines that define a point.

    The beginning and end points may or may not constitude the end points of a
    line segment, depending on the usage.

    """
    def __init__(self, beg, end):
        self.beg = beg
        self.end = end
    def __eq__(self, other):
        return type(self) == type(other) and \
            self.beg == other.beg and \
            self.end == other.end
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Line({}, {})'.format(self.beg, self.end)
    def to_vector(self):
        """Return Vector version of line from beginning to end."""
        return self.end - self.beg
    def get_angle(self):
        return self.to_vector().get_angle()
    def dist_to_point(self, point):
        """Return distance to a point.

        point (Point)  : point to calculate distance to
        return (Float) : distance from self to point

        """
        return _dist_point_to_line(point, self)
    def intersection_with_line(self, line):
        """Return the point of intersection with another line.

        line (Line)    : line to find intersction with
        return (Float) : point of intersection

        Throws NoIntersectionException if the lines do not intersect (are
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
        den = x1_x2*y3_y4 - y1_y2*x3_x4
        if den == 0.0:
            raise NoIntersectionException(
                'No intersection found between %s and %s' % (self, line))
        a, b = x1y2-y1x2, x3y4-y3x4
        P_x = (a*x3_x4 - x1_x2*b)/den
        P_y = (a*y3_y4 - y1_y2*b)/den
        return Point(P_x, P_y)


class Angle(object):
    """An angle, stored in both radians and degrees."""
    def __init__(self, value, is_degree):
        self.update_angle(value, is_degree)
    def __eq__(self, other):
        return type(self) == type(other) and \
            self.degree == other.degree and \
            self.radian == other.radian
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Angle({}, {})'.format(self.degree, True)
    def __add__(self, other):
        if type(other) == Angle:
            return self.normalize(self.radian + other.radian, False)
        raise TypeError(_type_error_str(self, other))
    def __sub__(self, other):
        if type(other) == Angle:
            return self.normalize(self.radian - other.radian, False)
        raise TypeError(_type_error_str(self, other))
    def update_angle(self, value, is_degree):
        if is_degree:
            self.degree = value % 360
            self.radian = _deg_to_rad(self.degree)
        else:
            self.radian = value % (2*math.pi)
            self.degree = _rad_to_deg(self.radian)
    def normalize(self, value, is_degree):
        """Force a value to be in between 0-2pi radians/0-360 degrees."""
        max_value = 360 if is_degree else 2*math.pi
        return Angle(value % max_value, is_degree)
    def get_normalized_vector(self):
        """Return a normalized (magnitude 1) vector at this angle."""
        return Vector(math.cos(self.radian), math.sin(self.radian))


class NoIntersectionException(Exception):
    pass

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
    p1 = line.beg
    p2 = line.end
    num = abs((p2.y-p1.y)*p0.x - (p2.x-p1.x)*p0.y + p2.x*p1.y - p2.y*p1.x)
    den = math.sqrt((p2.y-p1.y)**2 + (p2.x-p1.x)**2)
    return num/den

delta = 0.0000001
def _close(x, y):
    return abs(x-y) < delta


# Testing
def test():
    p1 = Point(0.1, 0.3)
    v1 = Vector(0.4, -0.1)
    assert p1 + v1 == Point(0.5, 0.2)
    p2 = Point(-0.4, 0.3)
    assert p1.dist_to_point(p2) == 0.5
    assert p1 - p2 == Vector(0.5, 0.0)
    v2 = Vector(3.0, 4.0)
    assert v2.get_magnitude() == 5.0
    assert v2.get_normalized() == Vector(3.0/5.0, 4.0/5.0)
    v3 = Vector(-3.0, -4.0)
    assert v3.get_magnitude() == 5.0
    assert v3.get_normalized() == Vector(-3.0/5.0, -4.0/5.0)
    v4 = Vector(3.0, -4.0)
    assert v4.get_magnitude() == 5.0
    assert v4.get_normalized() == Vector(3.0/5.0, -4.0/5.0)
    v5 = Vector(1.0, 1.0)
    assert v5.get_angle() == Angle(45, True)
    l1 = Line(p1, p2)
    assert l1.dist_to_point(p1) == 0.0
    assert p1.dist_to_line(l1) == 0.0
    p3 = Point(-0.1, 0.5)
    assert l1.dist_to_point(p3) == 0.2
    assert p3.dist_to_line(l1) == 0.2
    l2 = Line(Point(0,0), Point(1,1))
    assert l2.to_vector() == Vector(1,1)
    assert l2.get_angle() == Angle(45, True)
    p4 = Point(0,1)
    assert l2.dist_to_point(p4) == 1/math.sqrt(2)
    assert p4.dist_to_line(l2) == 1/math.sqrt(2)
    l3 = Line(Point(2,0), Point(0,2))
    assert l2.intersection_with_line(l3) == Point(1,1)
    assert l3.intersection_with_line(l2) == Point(1,1)
    l4 = Line(Point(-2,0), Point(0,-2))
    assert l2.intersection_with_line(l4) == Point(-1,-1)
    assert l4.intersection_with_line(l2) == Point(-1,-1)
    l3 = Line(Point(-2,-2), Point(-3,-3))
    assert l3.to_vector() == Vector(-1,-1)
    assert l3.get_angle() == Angle(225, True)
    a1 = Angle(180, True)
    assert a1 == Angle(math.pi, False)
    assert a1 == Angle(3*math.pi, False)
    pos1 = Position(p4, a1)
    v6 = Vector(-2, 0.5)
    assert pos1 == Position(Point(0,1), Angle(180, True))
    assert pos1 + v6 == Position(Point(-2, 1.5), Angle(180, True))
    assert pos1 + v6 == Position(Point(-2, 1.5), a1)
    assert pos1 + a1 == Position(p4, Angle(0, True))
    assert pos1 + a1 == Position(p4, Angle(360, True))
    assert Angle(45, True) == Angle(360 + 45, True)
    assert a1.normalize(360+32, True) == Angle(32, True)
    print 'All assertions passed'


if __name__ == '__main__':
    test()
