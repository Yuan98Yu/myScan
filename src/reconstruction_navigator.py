import airsim
import time
import math
import os
import json

from abc import abstractmethod
from airsim.types import Vector3r
from utils import get_config_from_file


class ReconstructionNavigator:
    """Navigator for reconstruction with UAV in AirSim

    Attributes:
        conf: Configurations in JSON format.
    """

    def __init__(self, conf):
        """Initialization."""
        print("start initialization...")
        self._config = conf
        self._safety_surface = conf["safety_surface"]
        self._uav = airsim.MultirotorClient()
        self._base_height = self._uav.simGetGroundTruthKinematics().position.z_val
        self._uav.confirmConnection()
        self._uav.enableApiControl(True)
        print("Initialization finishes!")

    @abstractmethod
    def explore(self):
        """Explore the place of interest to get a rough mesh model."""
        pass

    @abstractmethod
    def exploit(self):
        """Exploit the place of interest to get an accurate mesh model."""
        pass

    def generate_explore_views(self):
        pass

    def _observe_at_view(self, view, img_path):
        """Fly to given position and take a photo at given orientation."""
        position = view["position"]
        position = (
            Vector3r(position[0], position[1], position[2])
            if type(position) == list
            else position
        )
        print(
            "Observe (%f, %f, %f) with yaw %f and pitch %f"
            % (
                position.x_val,
                position.y_val,
                position.z_val,
                view["yaw"] / math.pi * 180,
                view["pitch"] / math.pi * 180,
            )
        )
        self._move_to(position, yaw=view["yaw"] / math.pi * 180)
        # self._uav.hoverAsync().join()
        # time.sleep(5)
        self._uav.simSetCameraOrientation(
            "0", airsim.to_quaternion(view["pitch"], 0.0, 0.0)
        )
        png = self._uav.simGetImage("0", airsim.ImageType.Scene)
        ReconstructionNavigator.mkdir(os.path.dirname(img_path))
        airsim.write_file(img_path, png)

    def _move_to(self, target, speed=5.0, yaw=0.0):
        """Due to bugs in controlling the drone(issue 1677 and 1292), the temporal solution is presented."""

        target = (
            Vector3r(target[0], target[1], target[2])
            if type(target) == list
            else target
        )
        self._uav.moveToPositionAsync(
            target.x_val,
            target.y_val,
            target.z_val,
            speed,
            yaw_mode=airsim.YawMode(False, yaw),
        ).join()
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
        current_dir = ""
        for directory in os.path.split(dir_path):
            current_dir = os.path.join(current_dir, directory)
            if not os.path.exists(current_dir):
                os.mkdir(current_dir)
