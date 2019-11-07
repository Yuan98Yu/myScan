from airsim.types import Vector3r
import math
import os
import sympy
import matplotlib.pyplot as plt

from dete import DETE, Point, Triangle
from utils import is_poi_within_poly, vector_to_yaw_and_pitch, point2list


class DeteCircle(DETE):
    def explore(self):
        """Explore the place of interest from the outer circle to the inner circle"""
        if not self._safety_surface["type"] == "circle":
            return
        img_dir = self._config["image_dir"]
        # Arm and take off
        self._uav.armDisarm(True)
        self._uav.takeoffAsync().join()
        self._move_to(Vector3r(0, 0, -60))
        # Explore center point
        points_per_round = self.generate_explore_views()
        self.show_explore_path(points_per_round)
        center = points_per_round[0][0]
        root_triangle = Triangle(center, center, center, None, True)
        photo_counter = 1
        print("Center point, counter=1")
        self.explore_point(
            center, root_triangle, root_triangle.pc, os.path.join(img_dir, "1.png")
        )
        # Explore the outer circle
        for point in points_per_round[-1]:
            photo_counter += 1
            print(
                "round %d, point, counter=%d"
                % (len(points_per_round) - 1, photo_counter)
            )
            self.explore_point(
                point,
                root_triangle,
                root_triangle.pc,
                os.path.join(img_dir, "%d.png") % photo_counter,
            )
        # Explore other points from the outer circle to the inner circle
        for i in range(len(points_per_round) - 2, 0, -1):
            for point in points_per_round[i]:
                photo_counter += 1
                print("round %d, point, counter=%d" % (i, photo_counter))
                tmp_triangle = self.__generate_smallest_triangle(
                    point, center, points_per_round[i + 1]
                )
                self.explore_point(
                    point,
                    tmp_triangle,
                    tmp_triangle.pc,
                    os.path.join(img_dir, "%d.png" % photo_counter),
                )

    def generate_explore_views(self):
        """Generate points on concentric circles.
        The number of points is given by self._config, the number of round is calculated with point_number
        """
        # read config from json
        center_point = self._safety_surface["center"]
        radius = self._safety_surface["radius"]
        point_number = self._config["point_number"]
        # declare function's output: points_per_round
        points_per_round = list()
        points_per_round.append(
            [Point(center_point[0], center_point[1], center_point[2])]
        )
        # calculate circle_num from point_number
        circle_num = sympy.Symbol("circle_num")
        num_list = sympy.solve(
            math.pi * (1 + circle_num) * (circle_num) - point_number, circle_num
        )
        for value in num_list:
            if value > 0:
                circle_num = int(math.ceil(value))
                print("circle_num: %d" % circle_num)
                break
        # calculate points_num_per_round from radius, point_number, circle_num
        points_num_per_round = [1]
        total_points_num = 1
        for i in range(1, circle_num + 1):
            points_num_per_round.append(int(math.ceil(2 * math.pi * i)))
            total_points_num += points_num_per_round[i]
        print("total: points_num: %d" % total_points_num)
        # generate points' coordinate for per round
        for i in range(1, circle_num + 1):
            tmp_radius = radius * i / circle_num
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
        """minor change to DETE.explore_point(...)"""
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
        self._move_to(p.vec3)
        print("\n")

    @staticmethod
    def __generate_smallest_triangle(point, center, points_list):
        """ get the three vertices(from points have been explored) of 
        the smallest triangle that surrouding the specified point.         
        Point is a list of [x,y,z], but only use [x,y] in this function.

        Args:
            point: Point  the point to be surrounded
            center: Point  the fixed point of the triangle
            points_list: List[Point]  the list to find the other two points
        Returns:
            smallest_triangle: Triangle
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
        plt.savefig("./dete_circle_path.png")


if __name__ == "__main__":
    from utils import get_config_from_file

    print("start test".center(100, "*"))
    dete = DeteCircle(get_config_from_file("../configs/dete_circle.json"))
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
