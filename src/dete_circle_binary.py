import sympy
import os
import math
import matplotlib.pyplot as plt
from airsim.types import Vector3r

from dete import DETE, Triangle, Point
from utils import (
    generate_smallest_triangle,
    point2list,
    is_poi_within_poly,
    is_ray_intersects_segment,
    vector_to_yaw_and_pitch,
)



class DeteCircleBinary(DETE):
    def explore(self):
        if not self._safety_surface["type"] == "circle":
            return

        img_dir = self._config["image_dir"]
        # Arm and takeoff
        self._uav.armDisarm(True)
        self._uav.takeoffAsync().join()
        self._move_to(Vector3r(0, 0, -60))
        # Explore center point and points on edge
        points_per_round = self.generate_explore_views()
        # self.show_explore_path(points_per_round)

        center = points_per_round[0][0]
        root_triangle = Triangle(center, center, center, None, True)
        point_counter = 1
        print("Center point, counter=1")
        self.explore_point(
            center,
            root_triangle,
            root_triangle.pc,
            os.path.join(img_dir, "%d.png" % point_counter),
        )

        for point in points_per_round[-1]:
            point_counter += 1
            print(
                "round %d, point, counter=%d"
                % (len(points_per_round) - 1, point_counter)
            )
            self.explore_point(
                point,
                root_triangle,
                root_triangle.pc,
                os.path.join(img_dir, "%d.png" % point_counter),
            )

        """0 1 2 3 4,2在0和4中间,1在0和2中间3在2和4中间"""
        point_counter = self.between(0, 4, points_per_round, 2, point_counter, img_dir)
        point_counter = self.between(0, 2, points_per_round, 1, point_counter, img_dir)
        point_counter = self.between(2, 4, points_per_round, 3, point_counter, img_dir)

    def between(self, former, latter, points_per_round, round, point_counter, img_dir):
        for point in points_per_round[round]:
            mindis = 1000000
            tmpoint = point
            for bpoint in points_per_round[former]:
                tmpdis = (bpoint.x - point.x) ** 2 + (bpoint.y - point.y) ** 2
                if tmpdis < mindis:
                    tmpoint = bpoint
                    mindis = tmpdis
            point_counter += 1
            print("round %d, point_counter=%d" % (round, point_counter))
            tmp_triangle = self.__generate_smallest_triangle(
                point, tmpoint, points_per_round[latter]
            )
            self.explore_point(
                point,
                tmp_triangle,
                tmp_triangle.pc,
                os.path.join(img_dir, "%d.png" % point_counter),
            )
        return point_counter

    def generate_explore_views(self):
        """
        
        """
        # read config from json
        center_point = self._safety_surface["center"]
        radius = self._safety_surface["radius"]
        point_num = self._config["point_num"]
        
        # function's output: points_per_round
        points_per_round = list()
        tmp_list = list()
        tmp_list.append(Point(center_point[0], center_point[1], center_point[2]))
        points_per_round.append(tmp_list)

        # calculate delta_len from photo_num, radius
        delta_len = sympy.Symbol("x")
        rm_point_num = point_num - 1
        round_num = 4
        num_list = sympy.solve(rm_point_num * delta_len - 5 * math.pi * radius, delta_len)
        # the output of sympy.solve() is a list[length: 2], we choose the one bigger than 0
        for value in num_list:
            if value > 0:
                delta_len = int(math.ceil(value))
                break

        # calculate points_num_per_round from radius, point_num, round_num
        points_num_per_round = [1]
        total_points_num = 1
        for i in range(1, round_num + 1):
            points_num_per_round.append(
                int(math.ceil(2 * math.pi * i * radius / 4 / delta_len))
            )
            total_points_num += points_num_per_round[i]
        print("total: points_num: %d" % total_points_num)

        # generate points' coordinate for per round
        for i in range(1, round_num + 1):
            tmp_radius = radius * i / 4
            delta_theta = 2 * math.pi / points_num_per_round[i]
            tmp_list = list()
            for num in range(points_num_per_round[i]):
                theta = num * delta_theta
                tmp_list.append(
                    Point(
                        center_point[0] + tmp_radius * math.cos(theta),
                        center_point[1] + tmp_radius * math.sin(theta),
                        center_point[2],
                    )
                )
            points_per_round.append(tmp_list)
        return points_per_round

    def explore_point(self, p, parent_triangle, center_pc, img_path):
        print("Prepare to explore (%f, %f, %f)" % (p.x, p.y, p.z))
        extra_height = 80
        self._move_to(p.vec3)
        print("p.h: %f" % p.h)
        p.h = self._drop_till_obstacle_detected()
        print("The highest obstacle's height is %f" % p.h)
        if parent_triangle.is_root:
            camera = vector_to_yaw_and_pitch([0, 0, 1])
            camera.update({"position": [p.x, p.y, p.h - extra_height]})
        else:
            print(
                "Height of parent triangle: %f, %f, %f"
                % (parent_triangle.p1.h, parent_triangle.p2.h, parent_triangle.p3.h)
            )
            obstacle_p1 = [
                parent_triangle.p1.x,
                parent_triangle.p1.y,
                parent_triangle.p1.h,
            ]
            obstacle_p2 = [
                parent_triangle.p2.x,
                parent_triangle.p2.y,
                parent_triangle.p2.h,
            ]
            obstacle_p3 = [
                parent_triangle.p3.x,
                parent_triangle.p3.y,
                parent_triangle.p3.h,
            ]
            camera = DETE.cal_camera(
                [p.x, p.y, p.h], obstacle_p1, obstacle_p2, obstacle_p3
            )
        self._observe_at_view(camera, img_path)
        if -p.h < extra_height:
            print("explore extra pictures:")
            camera_1 = {"pitch": camera["pitch"], "yaw": math.pi + camera["yaw"]}
            camera_1.update({"position": [p.x, p.y, (p.h - extra_height + p.z) / 2]})
            # print("extra_path_1:%s" % "%s_%d.png" % (img_path[:-4], 1))
            self._observe_at_view(camera_1, "%s_%d.png" % (img_path[:-4], 1))
            camera_2 = vector_to_yaw_and_pitch([0, 0, 1])
            camera_2.update({"position": [p.x, p.y, p.z]})
            self._observe_at_view(camera_2, "%s_%d.png" % (img_path[:-4], 2))
        self._move_to(p.vec3)
        print("\n")

    @staticmethod
    def __generate_smallest_triangle(point, center, points_list):
        """
            point is a list of [x,y,z],but only use [x,y] in this function
            input:
                point:the point to be surrounded
                center:the fixed point of the triangle
                points_list:the list to find the other two points
            output:
                smallest_triangle:
        """
        # search for the two nearest points
        tmp_point = point2list(point)
        tmp_center = point2list(center)
        list.sort(
            points_list, key=lambda p: (p.x - point.x) ** 2 + (p.y - point.y) ** 2
        )
        p1, p2 = points_list[0], points_list[1]
        for i in range(len(points_list)):
            for j in range(i + 1, len(points_list)):
                p1, p2 = points_list[i], points_list[j]
                if is_poi_within_poly(
                    tmp_point, [[point2list(p1), point2list(p2), tmp_center]]
                ):
                    break
        return Triangle(center, p1, p2)

    @staticmethod
    def show_explore_path(points_per_round):
        all_points = list()
        for points_list in points_per_round:
            all_points.extend(points_list)
        x = [p.x for p in all_points]
        y = [p.y for p in all_points]
        l = plt.plot(x, y)
        plt.setp(l, markersize=1)
        plt.savefig("./binary_path.png")


if __name__ == "__main__":
    from utils import get_config_from_file

    print("start test".center(100, "*"))
    dete = DeteCircleBinary(
        get_config_from_file("../configs/dete_circle_binary_extra.json")
    )
    dete.explore()
    # print("test of change h".center(100, "*"))
    # l = [Point(1,1,1), Point(0,0,0)]
    # center = Point(0,0,0)
    # def change_h(p_list, point):
    #     list.sort(p_list, key=lambda p: (p.x - point.x) ** 2 + (p.y - point.y) ** 2)
    #     for p in p_list:
    #         p.h = -1
    # change_h(l, center)
    # for p in l:
    #     print(p.h)
