from airsim.types import Vector3r
import math
import os

from reconstruction_navigator import ReconstructionNavigator


class OfflineNavigator(ReconstructionNavigator):
    def explore(self):
        """An offline method of exploration."""
        img_dir = self._config["image_dir"]
        # Generate flight plan
        views = self.generate_explore_views()
        # Arm and takeoff
        self._uav.armDisarm(True)
        self._uav.takeoffAsync().join()
        current_pos = self._uav.simGetGroundTruthKinematics().position
        self._move_to(Vector3r(current_pos.x_val, current_pos.y_val, -60))
        # Fly to each view and take photos
        for idx, view in enumerate(views):
            self._observe_at_view(view, os.path.join(img_dir, "%s.png" % idx))

    def exploit(self):
        pass

    def generate_explore_views(self):
        """Generate the explore views according to the poi.

        Currently, there are 2 diffent types of view_list
            for 2 diffent type of safety_surface: circle and cylinder
        Returns:
            A list of views each of which contains position, camera orientation and take photos here or not.
        """
        views = []
        if self._safety_surface["type"] == "circle":
            # Generate points evently distributed on the circle
            center = self._safety_surface["center"]
            center = Vector3r(center[0], center[1], center[2])
            x0 = center.x_val
            y0 = center.y_val
            z0 = center.z_val
            radius = self._safety_surface["radius"]
            TOTAL_NUM = self._config["point_num"]
            ROUND_NUM = self._config.get("round_num", 1)
            delta_theta = 2 * math.pi / (TOTAL_NUM / ROUND_NUM)

            for i in range(TOTAL_NUM):
                theta = delta_theta * i
                x = x0 + radius * math.sin(theta)
                y = y0 + radius * math.cos(theta)
                pitch = -45
                views.append(
                    {
                        "position": Vector3r(x, y, z0),
                        "yaw": -1 * (0.5 * math.pi + theta),
                        "pitch": pitch,
                    }
                )
        elif self._safety_surface["type"] == "cylinder":
            # Generate points spiral the cylinder
            top_center = self._safety_surface["top_center"]
            top_center = Vector3r(top_center[0], top_center[1], top_center[2])
            x0 = top_center.x_val
            y0 = top_center.y_val
            bottom = self._safety_surface.get("bottom", 0)
            height = top_center.z_val - bottom
            radius = self._safety_surface["radius"]
            TOTAL_NUM = self._config["point_num"]
            ROUND_NUM = self._config.get("round_num", 1)
            START_PITCH = self._config.get("start_pitch", -45)
            END_PITCH = self._config.get("end_pitch", 45)
            delta_theta = 2 * math.pi / (TOTAL_NUM / ROUND_NUM)
            delta_height = height / (TOTAL_NUM - 1)
            delta_pitch = (END_PITCH - START_PITCH) / TOTAL_NUM
            for i in range(TOTAL_NUM):
                theta = delta_theta * i
                x = x0 + radius * math.sin(theta)
                y = y0 + radius * math.cos(theta)
                z = bottom + i * delta_height
                pitch = START_PITCH + i * delta_pitch
                views.append(
                    {
                        "position": Vector3r(x, y, z),
                        "yaw": -1 * (0.5 * math.pi + theta),
                        "pitch": pitch / 180 * math.pi,
                    }
                )
        else:
            print(
                "OfflineNavigator: unknown type of safety_surface (%s)"
                % self._safety_surface["type"]
            )

        return views


if __name__ == "__main__":
    from utils import get_config_from_file

    config = get_config_from_file("./configs/config.json")
    rn = OfflineNavigator(config)
    rn.explore()
