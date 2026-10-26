#!/usr/bin/env python3
"""List validated scenario files using only stdlib and PyYAML."""
from pathlib import Path
from shopping_robot_sim.scenarios import load_scenario


def main():
    config = Path(__file__).resolve().parents[1]/'ros2_ws/src/shopping_robot_gazebo/config'
    for path in sorted((config/'scenarios').glob('*.yaml')):
        data = load_scenario(config, path.stem)
        pose = data['robot_pose']
        print(f"{path.stem}: robot=({pose['x']}, {pose['y']}, {pose['yaw']}); pedestrians={len(data['pedestrians'])}")


if __name__ == '__main__':
    main()
