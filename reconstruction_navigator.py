import airsim
import time
import math
import os
import json

from abc import abstractmethod
from airsim.types import Vector3r


class ReconstructionNavigator:
    """Navigator for reconstruction with UAV in AirSim

    Attributes:
        conf: Configurations in JSON format.
    """
    def __init__(self, conf):
        """Initialization."""
        self._config = conf
        self._safety_surface = conf['safety_surface']
        self._fov = conf.get('fov', 84) / 180 * math.pi
        self._config['input_model_path'] = self._config.get('input_mesh_path', self._config['output_mesh_path_1'])
        self._uav = airsim.MultirotorClient()
        self._base_height = self._uav.simGetGroundTruthKinematics().position.z_val
        self._uav.confirmConnection()
        self._uav.enableApiControl(True)

    @abstractmethod
    def explore(self):
        """Explore the place of interest to get a rough mesh model."""
        pass

    @abstractmethod
    def exploit(self):
        """Exploit the place of interest to get an accurate mesh model."""
        pass

    def offline_explore(self):
        """An offline method of exploration."""
        img_dir = self._config['image_dir_1']
        out_mesh_path = self._config['output_mesh_path_1']
        # Generate flight plan
        views = self.generate_explore_views()
        # Arm and takeoff
        self._uav.armDisarm(True)
        self._uav.takeoffAsync().join()
        current_pos = self._uav.simGetGroundTruthKinematics().position
        self._move_to(Vector3r(current_pos.x_val, current_pos.y_val, -60))
        # Fly to each view and take photos
        for idx, view in enumerate(views):
            self._observe_at_view(view, os.path.join(img_dir, '%s.png' % idx))

    def generate_explore_views(self):
        """Generate the explore views according to the poi.

        Returns:
            A list of views each of which contains position, camera orientation and take photos here or not.
        """
        views = []
        if self._safety_surface['type'] == 'cylinder':
            top_center = self._safety_surface['top_center']
            top_center = Vector3r(top_center[0], top_center[1], top_center[2])
            x0 = top_center.x_val
            y0 = top_center.y_val
            bottom = self._safety_surface.get('bottom', 0)
            height = top_center.z_val - bottom
            radius = self._safety_surface['radius']
            TOTAL_NUM = self._config['photo_num']
            ROUND_NUM = self._config.get('round_num', 1)
            START_PITCH = self._config.get('start_pitch', -45)
            END_PITCH = self._config.get('end_pitch', 45)
            delta_theta = 2 * math.pi / (TOTAL_NUM / ROUND_NUM)
            delta_height = height / (TOTAL_NUM - 1)
            delta_pitch = (END_PITCH - START_PITCH) / TOTAL_NUM
            for i in range(TOTAL_NUM):
                theta = delta_theta * i
                x = x0 + radius * math.sin(theta)
                y = y0 + radius * math.cos(theta)
                z = bottom + i * delta_height
                pitch = START_PITCH + i * delta_pitch
                views.append({
                    'position': Vector3r(x, y, z),
                    'yaw': -1 * (0.5 * math.pi + theta),
                    'pitch': pitch / 180 * math.pi
                })

        return views

    def _observe_at_view(self, view, img_path):
        """Fly to given position and take a photo at given orientation."""
        position = view['position']
        position = Vector3r(position[0], position[1], position[2]) if type(position) == list else position
        print("Observe (%f, %f, %f) with yaw %f and pitch %f" % (position.x_val, position.y_val, position.z_val,
                                                                      view['yaw'] / math.pi * 180,
                                                                      view['pitch'] / math.pi * 180))
        self._move_to(position, yaw=view['yaw'] / math.pi * 180)
        # self._uav.hoverAsync().join()
        # time.sleep(5)
        self._uav.simSetCameraOrientation('0', airsim.to_quaternion(view['pitch'], 0.0, 0.0))
        png = self._uav.simGetImage('0', airsim.ImageType.Scene)
        ReconstructionNavigator.mkdir(os.path.dirname(img_path))
        airsim.write_file(img_path, png)

    def _move_to(self, target, speed=5.0, yaw=0.0):
        """Due to bugs in controlling the drone(issue 1677 and 1292), the temporal solution is presented."""

        target = Vector3r(target[0], target[1], target[2]) if type(target) == list else target
        self._uav.moveToPositionAsync(target.x_val, target.y_val, target.z_val, speed, yaw_mode=airsim.YawMode(
            False, yaw)).join()
        # current_pos = self._uav.simGetGroundTruthKinematics().position
        # distance = (target - current_pos).get_length()
        # if distance == 0:
        #     return
        # duration = distance / speed
        # velocity = (target - current_pos) / duration
        # self._uav.moveByVelocityAsync(velocity.x_val, velocity.y_val, velocity.z_val, duration,
        #                               drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        #                               yaw_mode=airsim.YawMode(False, yaw))
        # time.sleep(duration)
        # self._uav.moveByVelocityAsync(0, 0, 0, 3, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        #                               yaw_mode=airsim.YawMode(False, yaw))

    @staticmethod
    def mkdir(dir_path):
        """Creat directories if not exist."""
        current_dir = ''
        for directory in os.path.split(dir_path):
            current_dir = os.path.join(current_dir, directory)
            if not os.path.exists(current_dir):
                os.mkdir(current_dir)

    @staticmethod
    def get_config_from_file(config_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)


if __name__ == '__main__':
    # config = ReconstructionNavigator.get_config_from_file('./configs/config-cylinder.json')
    config = ReconstructionNavigator.get_config_from_file('./configs/config.json')
    rn = ReconstructionNavigator(config)
    rn.offline_explore()
