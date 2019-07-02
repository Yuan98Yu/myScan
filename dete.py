import math
import os
import matplotlib.pyplot as plt

from utils import *
from reconstruction_navigator import ReconstructionNavigator
from airsim.types import Vector3r


class Point:
    def __init__(self, x, y, z, h=100):
        self.x = x
        self.y = y
        self.z = z
        self.h = h

    @property
    def lst(self):
        return [self.x, self.y, self.z]

    @property
    def vec3(self):
        return Vector3r(self.x, self.y, self.z)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    def __truediv__(self, other):
        return Point(self.x / other, self.y / other, self.z / other)


class Triangle:
    def __init__(self, p1, p2, p3, parent=None, is_root=False):
        self.p1 = Point(p1[0], p1[1], p1[2]) if type(p1) == list else p1
        self.p2 = Point(p2[0], p2[1], p2[2]) if type(p2) == list else p2
        self.p3 = Point(p3[0], p3[1], p3[2]) if type(p3) == list else p3
        self.is_root = is_root
        self.parent = parent
        self.childs = []
        self.pc = (self.p1 + self.p2 + self.p3) / 3
        if self.parent is not None and self.parent.is_root:
            divide_ratio = 8
            self.pc = self.p1 / (divide_ratio / (divide_ratio - 1)) + (self.p2 + self.p3) / (2 * divide_ratio)

    def fissure(self, num=3):
        if self.is_root:
            return
        if num > 0:
            self.add_child(self.pc, self.p1, self.p2)
        if num > 1:
            self.add_child(self.pc, self.p2, self.p3)
        if num > 2:
            self.add_child(self.pc, self.p3, self.p1)

    def grow(self):
        if len(self.childs) == 0:
            self.fissure()
        else:
            for triangle in self.childs:
                triangle.grow()

    def add_child(self, p1, p2, p3):
        child = Triangle(p1, p2, p3, self)
        self.childs.append(child)

    def dfs(self):
        triangles = []
        if not self.is_root:
            triangles.append(self)
        for child in self.childs:
            triangles.extend(child.dfs())
        return triangles


class DETE(ReconstructionNavigator):
    def explore(self):
        if not self._safety_surface['type'] == 'circle':
            return
        img_dir = self._config['image_dir_1']
        out_mesh_path = self._config['output_mesh_path_1']
        # Arm and takeoff
        self._uav.armDisarm(True)
        self._uav.takeoffAsync().join()
        self._move_to(Vector3r(0, 0, -60))
        # Explore center point and points on edge
        root = self.generate_explore_views()
        DETE.show_explore_path(root)
        center = root
        triangles = root.dfs()
        # all_points = [center.pc, center.childs[0].p2]
        print("Center point, counter=1")
        self.explore_point(center.pc, center, center.pc, os.path.join(img_dir, '1.png'))
        print("Edge point, counter=2")
        photo_counter = 2
        self.explore_point(center.childs[0].p2, center.childs[0], center.pc, os.path.join(img_dir, '2.png'))
        for triangle in triangles:
            if triangle.parent.is_root:
                # all_points.extend([triangle.p3, triangle.pc])
                photo_counter += 1
                print("Edge point, counter=%d" % photo_counter)
                self.explore_point(triangle.p3, triangle, center.pc, os.path.join(img_dir, '%d.png' % photo_counter))
            photo_counter += 1
            print("Center point, counter=%d" % photo_counter)
            self.explore_point(triangle.pc, triangle, center.pc, os.path.join(img_dir, '%d.png' % photo_counter))

    def explore_point(self, p, parent_triangle, center_pc, img_path):
        print("Prepare to explore (%f, %f, %f)" % (p.x, p.y, p.z))
        extra_height = 80
        self._move_to(p.vec3)
        p.h = self._drop_till_obstacle_detected()
        print("The highest obstacle's height is %f" % p.h)
        if parent_triangle.is_root:
            camera = vector_to_yaw_and_pitch([0, 0, 1])
            camera.update({'position': [p.x, p.y, p.h - extra_height]})
        elif parent_triangle.pc.x == p.x and parent_triangle.pc.y == p.y and parent_triangle.pc.z == p.z:
            print("Height of parent triangle: %f, %f, %f" % (parent_triangle.p1.h, parent_triangle.p2.h,
                                                             parent_triangle.p3.h))
            obstacle_p1 = [parent_triangle.p1.x, parent_triangle.p1.y, parent_triangle.p1.h]
            obstacle_p2 = [parent_triangle.p2.x, parent_triangle.p2.y, parent_triangle.p2.h]
            obstacle_p3 = [parent_triangle.p3.x, parent_triangle.p3.y, parent_triangle.p3.h]
            camera = DETE.cal_camera([p.x, p.y, p.h], obstacle_p1, obstacle_p2, obstacle_p3)
        elif parent_triangle.parent.is_root:
            fov = self._config['fov']
            h = self._safety_surface['center'][2] * -1
            R = self._safety_surface['radius']
            pitch = -1 * (0.5 * fov * math.pi / 180 + math.atan(0.5 * h * R))
            yaw = vector_to_yaw_and_pitch((center_pc - p).lst)['yaw']
            position = [p.x, p.y, p.h - extra_height]
            camera = {
                'pitch': pitch, 'yaw': yaw, 'position': position
            }
        self._observe_at_view(camera, img_path)
        self._move_to(p.vec3)
        print("\n")

    def exploit(self):
        img_dir = self._config['image_dir_2']
        input_mesh_path = self._config['input_mesh_path']
        output_mesh_path = self._config['output_mesh_path_2']

    def generate_explore_views(self):
        center_point = self._safety_surface['center']
        radius = self._safety_surface['radius']
        photo_num = self._config['photo_num']
        round_num = self._config['round_num']
        root = Triangle(center_point, center_point, center_point, None, True)
        # Generate triangles in first round
        remaining_triangle_num = photo_num
        points_num_first_round = math.ceil((photo_num - 1) * 2 / (3**round_num + 1))
        delta_theta = 2 * math.pi / points_num_first_round
        points_first_round = []
        for i in range(points_num_first_round):
            theta = i * delta_theta
            points_first_round.append(Point(center_point[0] + radius * math.cos(theta),
                                            center_point[1] + radius * math.sin(theta), center_point[2]))
        for i in range(points_num_first_round - 1):
            root.add_child(root.pc, points_first_round[i], points_first_round[i + 1])
            remaining_triangle_num -= 1
        root.add_child(root.pc, points_first_round[-1], points_first_round[0])
        # Generate other triangles
        for i in range(1, round_num):
            root.grow()
        return root

    def _drop_till_obstacle_detected(self, speed=5.0):
        """Move downward vertically untill an obstacle is detected by lidar sensor."""
        current_pos = self._uav.simGetGroundTruthKinematics().position
        self._uav.moveByVelocityAsync(0, 0, speed, duration=1000)
        while True:
            try:
                lidar_data = self._uav.getLidarData(lidar_name='BottomObstacleDetector')
                if len(lidar_data.point_cloud) >= 3:
                    uav_pos = self._uav.simGetGroundTruthKinematics().position
                    for i in range(0, len(lidar_data.point_cloud), 3):
                        p = Vector3r(lidar_data.point_cloud[i], lidar_data.point_cloud[i + 1],
                                     lidar_data.point_cloud[i + 2], )
                        delta_height = p.z_val - uav_pos.z_val
                        inside_circle = Vector3r(p.x_val, p.y_val, 0).distance_to(Vector3r(uav_pos.x_val,
                                                                                           uav_pos.y_val, 0)) < 2
                        if delta_height > 0 and inside_circle:
                            self._uav.moveByVelocityAsync(0, 0, 0, duration=1.0)
                            return p.z_val
            except AttributeError:
                print("AttributeError")
                continue

    @staticmethod
    def cal_camera(p, p1, p2, p3, extra_height=80):
        """Calculate the position and orientation of photographing according to a center point p and other three
        points."""
        plane = plane_from_3_points(p1, p2, p3)
        intersection = intersection_of_line_and_plane(p, [0, 0, -1], p1, plane[:-1])
        is_convex_surface = True if p[2] - intersection[2] <= 0 else False
        if is_convex_surface:
            camera_pos = add(p, [0, 0, -1 * extra_height])
        else:
            d = distance(p, intersection)
            if d < extra_height / 2:
                camera_pos = add(p, [0, 0, -1 * extra_height])
            else:
                camera_pos = add(p, [0, 0, -2 * d])
        camera = vector_to_yaw_and_pitch(angular_bisector(camera_pos, p1, p2, p3))
        camera.update({'position': camera_pos})
        return camera

    @staticmethod
    def show_explore_path(root):
        center = root
        triangles = root.dfs()
        all_points = [center.pc, center.childs[0].p2]
        for triangle in triangles:
            if triangle.parent.is_root:
                all_points.extend([triangle.p3, triangle.pc])
            else:
                all_points.extend([triangle.pc])
        x = [p.x for p in all_points]
        y = [p.y for p in all_points]
        l = plt.plot(x, y, 'ro')
        l = plt.plot(x, y)
        plt.setp(l, markersize=1)
        plt.savefig('./path.png')


if __name__ == "__main__":
    dete = DETE(DETE.get_config_from_file("./configs/config.json"))
    dete.explore()

