import math


def vector3(p1, p2):
    return [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]


def length(vec3):
    return (vec3[0]**2 + vec3[1]**2 + vec3[2]**2)**0.5


def distance(v1, v2):
    return length(sub(v1, v2))


def norm(vec3):
    l = length(vec3)
    return [vec3[0] / l, vec3[1] / l, vec3[2] / l]


def included_angle(v1, v2):
    return math.acos(mult(v1, v2) / (length(v1) * length(v2)))


def add(*vec3s):
    result = [0, 0, 0]
    for v in vec3s:
        result[0] += v[0]
        result[1] += v[1]
        result[2] += v[2]
    return result


def sub(*vec3s):
    result = [vec3s[0][0], vec3s[0][1], vec3s[0][2]]
    for i in range(1, len(vec3s)):
        result[0] -= vec3s[i][0]
        result[1] -= vec3s[i][1]
        result[2] -= vec3s[i][2]
    return result


def mult(v1, v2):
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]


def mult_r(vec3, r):
    return [vec3[0] * r, vec3[1] * r, vec3[2] * r]


def divide(vec3, r):
    return [vec3[0] / r, vec3[1] / r, vec3[2] / r]


def plane_from_3_points(p1, p2, p3):
    """Return 4 arguments of the plane from 3 vertexes which locate on the plane."""
    a = (p2[1] - p1[1]) * (p3[2] - p1[2]) - (p3[1] - p1[1]) * (p2[2] - p1[2])
    b = (p2[2] - p1[2]) * (p3[0] - p1[0]) - (p3[2] - p1[2]) * (p2[0] - p1[0])
    c = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])
    d = 0 - (a * p1[0] + b * p1[1] + c * p1[2])
    return [a, b, c, d]


def intersection_of_line_and_plane(l0, l, p0, n):
    """Calculate the intersection according to https://en.wikipedia.org/wiki/Line–plane_intersection"""
    d = (mult(sub(p0, l0), n)) / mult(l, n)
    return add(mult_r(l, d), l0)


def angular_bisector(p, p1, p2, p3):
    """Return the vector of angular bisector formed by a center point p and other three points."""
    return norm(add(norm(vector3(p, p1)), norm(vector3(p, p2)), norm(vector3(p, p3))))


def vector_to_yaw_and_pitch(vec3):
    """Convert an orientation vector to yaw and pitch in radian."""
    if vec3[2] == 0:
        pitch = 0
    else:
        pitch_angle = included_angle([1, 0, 0], [vec3[0], 0, vec3[2]])
        pitch = pitch_angle if vec3[2] < 0 else -1 * pitch_angle

    if vec3[0] == 0 and vec3[1] == 0:
        yaw = 0
    else:
        yaw_angle = included_angle([1, 0, 0], [vec3[0], vec3[1], 0])
        yaw = yaw_angle if vec3[1] < 0 else -1 * yaw_angle
    return {
        'pitch': pitch,
        'yaw': yaw
    }


