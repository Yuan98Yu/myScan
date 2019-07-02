from dete import *
from utils import generate_smallest_triangle, point2list
import sympy


class DeteCircle(DETE):
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
        points_per_round = self.generate_explore_views()
        center = points_per_round[0]
        root_triangle = Triangle(center, center, center, None, True)
        photo_counter = 1
        print("Center point, counter=1")
        self.explore_point(root_triangle.pc, center, root_triangle.pc, os.path.join(img_dir, '1.png'))
        for point in points_per_round[1]:
            photo_counter += 1
            print("Edge point, counter=%d" % photo_counter)
            self.explore_point(root_triangle.pc, point, root_triangle.pc,
                               os.path.join(img_dir, '%d.png') % photo_counter)
        # Explore other points
        for i in range(len(points_per_round), 0, -1):
            for point in points_per_round[i]:
                photo_counter += 1
                print("Edge point, counter=%d" % photo_counter)
                tmp_triangle = self.__generate_smallest_triangle(point, center, points_per_round[i-1])
                self.explore_point(point, tmp_triangle, tmp_triangle.pc,
                                   os.path.join(img_dir, '%d.png' % photo_counter))

    def generate_explore_views(self):
        # read config from json
        center_point = self._safety_surface['center']
        radius = self._safety_surface['radius']
        photo_num = self._config['photo_num']
        # function's output: points_per_round
        points_per_round = list()
        points_per_round.append(Point(center_point[0], center_point[1], center_point[2]))
        # calculate round_num from photo_num, radius
        round_num = sympy.Symbol('round_num')
        num_list = sympy.solve(2 * math.pi * (1 + round_num) * (round_num) - photo_num, round_num)
        for value in num_list:
            if value > 0:
                round_num = int(math.ceil(value))
                break
        # calculate points_num_per_round from radius, photo_num, round_num
        points_num_per_round = [1]
        for i in range(1, round_num):
            points_num_per_round.append(int(math.ceil(2*math.pi*i)))
        # generate points' coordinate for per round
        for i in range(1, round_num):
            tmp_radius = radius * i/round_num
            delta_theta = 2 * math.pi / points_num_per_round[i]
            tmp_list = list()
            for num in range(points_num_per_round[i]):
                theta = num * delta_theta
                tmp_list.append(Point(center_point[0] + tmp_radius * math.cos(theta),
                                        center_point[1] + tmp_radius * math.sin(theta), center_point[2]))
            points_per_round.append(tmp_list)
        return points_per_round

    def __generate_smallest_triangle(self, point, center, points_list):
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
        point = point2list(point)
        center = point2list(center)
        points_list = [point2list(p) for p in points_list]
        return Triangle(generate_smallest_triangle(point, center, points_list))


if __name__ == "__main__":
    point1 = Point(1, 2, 3)
    point2 = Point(2, 2, 2)
    points_list = [point1, point2]
    points_list = [point2list(p) for p in points_list]
    for point in points_list:
        print(point)
    dete = DeteCircle(DeteCircle.get_config_from_file("./configs/dete_circle.json"))
    dete.explore()
