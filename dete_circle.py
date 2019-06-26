from dete import *


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


    def exploit(self):
        img_dir = self._config['image_dir_2']
        input_mesh_path = self._config['input_mesh_path']
        output_mesh_path = self._config['output_mesh_path_2']

    def generate_explore_views(self):
        center_point = self._safety_surface['center']
        radius = self._safety_surface['radius']
        photo_num = self._config['photo_num']
        round_num = self._config['round_num']
        points_num_per_round = []
        points_per_round = []
        base = 0
        for i in range(round_num+1):
            base += i**2
        for i in range(1, round_num):
            points_num_per_round.append(photo_num * i**2/base)
        root = Triangle(center_point, center_point, center_point, None, True)
        # Generate triangles in first round
        remaining_triangle_num = photo_num
        points_num_first_round = points_num_per_round[-1]
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
            tmp_radius = radius * i/round_num
            for num in range(points_num_per_round[i]):
                theta = num * delta_theta
                points_first_round.append(Point(center_point[0] + tmp_radius * math.cos(theta),
                                                center_point[1] + tmp_radius * math.sin(theta), center_point[2]))
        return root

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

