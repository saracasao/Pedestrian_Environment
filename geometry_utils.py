import math

OVERPERCENT_FEET = 0.05
OVERPERCENT_HEAD = 0.05

# ----------------3D UTILS----------------


class Point3D:
    """
    Represents a 3d point.
    """

    def __init__(self, x, y, z=0., s=1.):
        self.x = x
        self.y = y
        self.z = z
        self.s = s

    def getXYZ(self):
        return float(self.x) / self.s, float(self.y) / self.s, self.z

    def getAsXY(self):
        return float(self.x) / self.s, float(self.y) / self.s

    def normalize(self, dist=1.):
        return Point3D(self.x, self.y, self.z, math.sqrt(self.x ** 2 + self.y ** 2) / dist)


class Cylinder:
    def __init__(self, center, width, height):
        self.center = center
        self.width = width
        self.height = height

    @classmethod
    def XYZWH(cls, x, y, z, w, h):
        return cls(Point3D(x, y, z), w, h)

    def getCenter(self):
        return self.center

    def getWidth(self):
        return self.width

    def getHeight(self):
        return self.height

    def getXYZWH(self):
        x, y, z = self.getCenter().getXYZ()
        return x, y, z, self.getWidth(), self.getHeight()

    def getFeet(self):
        x, y, z = self.getCenter().getXYZ()
        zFeet = z + 0.05 * z

        return Point3D(x, y, zFeet)

    def getHair(self):
        x, y, z = self.getCenter().getXYZ()
        zHair = z - self.height
        return Point3D(x, y, zHair)


# ----------------2D UTILS----------------
class Point2D:
    """
    Represents a 2d point.
    Can transform between normal (x,y) and homogeneous (x,y,s) coordinates
    """

    def __init__(self, x, y, s=1.):
        self.x = x
        self.y = y
        self.s = s

    def getAsXY(self):
        return float(self.x) / self.s, float(self.y) / self.s

    def getAsXYS(self):
        return self.x, self.y, self.s

    def normalize(self, dist=1.):
        return Point2D(self.x, self.y, math.sqrt(self.x ** 2 + self.y ** 2) / dist)

    def multiply(self, val):
        return Point2D(self.x, self.y, self.s / val)


class Bbox:
    """
    Represents a bounding box (region) on a plane.
    Can transform between 'opposite corners' [(xmin, ymin), (xmax, ymax)] and 'size' [(xmin, ymin), height, width] coordinates
    """

    def __init__(self, xmin, xmax, width, ymin, ymax, height):
        self.xmin = int(xmin)
        self.xmax = int(xmax)
        self.width = int(width)
        self.ymin = int(ymin)
        self.ymax = int(ymax)
        self.height = int(height)

    @classmethod
    def FeetWH(cls, feet, width, height, heightReduced=False):
        if heightReduced:
            height /= (1 - 2 * OVERPERCENT_FEET)

        bx, by = feet.getAsXY()
        feetHeight = height * OVERPERCENT_FEET
        return cls(bx - width / 2, bx + width / 2, width, by + feetHeight - height, by + feetHeight, height)

    def getAsXmYmXMYM(self):
        return self.xmin, self.ymin, self.xmax, self.ymax


# ----------------Geometry UTILS----------------


def f_euclidian_image(a, b):
    """
    returns the euclidian distance between the two points
    """
    ax, ay = a.getAsXY()
    bx, by = b.getAsXY()
    return math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)


def f_add(a, b):
    """
    return addition of points a+b
    """
    return Point2D(b.s * a.x + a.s * b.x, b.s * a.y + a.s * b.y, a.s * b.s)


def f_subtract_ground(a, b):
    """
    return difference of points a-b
    """
    return Point3D(b.s * a.x - a.s * b.x, b.s * a.y - a.s * b.y, b.z, a.s * b.s)


def f_add_ground(a, b):
    """
    return addition of points a+b
    """
    return Point3D(b.s * a.x + a.s * b.x, b.s * a.y + a.s * b.y, b.z, a.s * b.s)