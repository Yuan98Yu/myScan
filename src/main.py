from utils import get_config_from_file
from reconstruction_navigator import ReconstructionNavigator
from offline import OfflineNavigator
from dete import DETE
from dete_circle import DeteCircle
from dete_circle_binary import DeteCircleBinary
import argparse


def main():
    def m(t):
        return {
            "offline": OfflineNavigator,
            "dete": DETE,
            "dete_circle": DeteCircle,
            "dete_circle_binary": DeteCircleBinary,
        }.get(t)

    parser = argparse.ArgumentParser()
    parser.add_argument("--config_path", "-c", default="./configs/config.json")
    args = parser.parse_args()

    config_path = args.config_path
    conf = get_config_from_file(config_path)
    rn = m(conf["type"])(conf)
    rn.explore()


if __name__ == "__main__":
    main()
