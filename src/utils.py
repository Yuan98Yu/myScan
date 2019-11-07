import math
import json


def get_config_from_file(config_path):
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def vector3(p1, p2):
    return [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]


def length(vec3):
    return (vec3[0] ** 2 + vec3[1] ** 2 + vec3[2] ** 2) ** 0.5


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
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


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
    return {"pitch": pitch, "yaw": yaw}


def is_ray_intersects_segment(poi, s_poi, e_poi):  # [x,y] [lng,lat]
    # 输入：判断点，边起点，边终点，都是[lng,lat]格式数组
    if s_poi[1] == e_poi[1]:  # 排除与射线平行、重合，线段首尾端点重合的情况
        return False
    if s_poi[1] > poi[1] and e_poi[1] > poi[1]:  # 线段在射线上边
        return False
    if s_poi[1] < poi[1] and e_poi[1] < poi[1]:  # 线段在射线下边
        return False
    if s_poi[1] == poi[1] and e_poi[1] > poi[1]:  # 交点为下端点，对应spoint
        return False
    if e_poi[1] == poi[1] and s_poi[1] > poi[1]:  # 交点为下端点，对应epoint
        return False
    if s_poi[0] < poi[0] and e_poi[1] < poi[1]:  # 线段在射线左边
        return False

    xseg = e_poi[0] - (e_poi[0] - s_poi[0]) * (e_poi[1] - poi[1]) / (
        e_poi[1] - s_poi[1]
    )  # 求交
    if xseg < poi[0]:  # 交点在射线起点的左侧
        return False
    return True  # 排除上述情况之后


def is_poi_within_poly(poi, poly):
    """
    输入：点，多边形三维数组
    poly=[[[x1,y1],[x2,y2],……,[xn,yn],[x1,y1]],[[w1,t1],……[wk,tk]]] 三维数组
    可以先判断点是否在外包矩形内
    if not isPoiWithinBox(poi,mbr=[[0,0],[180,90]]): return False
    但算最小外包矩形本身需要循环边，会造成开销，本处略去
    """
    sinsc = 0  # 交点个数
    for epoly in poly:  # 循环每条边的曲线->each polygon 是二维数组[[x1,y1],…[xn,yn]]
        for i in range(len(epoly) - 1):  # [0,len-1]
            s_poi = epoly[i]
            e_poi = epoly[i + 1]
            if is_ray_intersects_segment(poi, s_poi, e_poi):
                sinsc += 1  # 有交点就加1

    return True if sinsc % 2 == 1 else False


def generate_smallest_triangle(point, center, points_list):
    """point is a list of [x,y,z],but only use [x,y] in this function
    input:
            point: the point to be surrounded
            center: the fixed point of the triangle
            points_list: the list to find the other two points
        return:
            smallest_triangle: list of [center, p1, p2]
    """
    # search for the two nearest points
    tmp_list = sorted(
        points_list, key=lambda p: (p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2
    )
    p1, p2 = tmp_list[0], tmp_list[1]
    for i in range(len(tmp_list)):
        for j in range(i + 1, len(tmp_list)):
            p1, p2 = tmp_list[i], tmp_list[j]
            if is_poi_within_poly(point, [[p1, p2, center]]):
                break
    return center, p1, p2


def point2list(point):
    return [point.x, point.y, point.z]


if __name__ == "__main__":
    print("test of generate_smallest_triangle".center(100, "*"))
    point = [1, 1, 1]
    center = [0, 0, 0]
    points_list = [[1, 1, 1], [0, 0, 0], [1, 2, 1], [2, 2, 1], [3, 2, 1]]
    triangle = generate_smallest_triangle(point, center, points_list)

