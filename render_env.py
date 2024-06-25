import numpy as np
import vamp
from lab3_utils import transform_to_vamp_coords
from matplotlib import pyplot as plt
from vamp import pybullet_interface as vpb
from pathlib import Path
from lab3_consts import home
from env_consts import cubes
from rtde_iface import rtdeRobot
# robot = rtdeRobot("192.168.0.10")

# cube reference point: [0.57, -0.015, 0.21]
# cube size: [0.12, 0.24, 0.24]
ref_point_config = [0.16554054617881775, -2.233336111108297, -1.523730754852295, -0.9727459710887452, 1.5914708375930786, 0.17196546494960785]

def calculate_center(points):
    return np.mean(points, axis=0)

def calculate_half_extents(points, center):
    half_extents = np.max(np.abs(points - center), axis=0)
    return half_extents

def calculate_euler_angles():
    # Assuming axis-aligned cuboid for simplicity
    return [0, 0, 0]


def calculate_cuboid(points):
    """
    return center, eurler angles, half_extents
    """
    center = calculate_center(points)
    half_extents = calculate_half_extents(points, center)
    euler_angles = calculate_euler_angles()
    return center, euler_angles, half_extents

urdf_path = str(Path(__file__).parent / "ur5" / "ur5_spherized.urdf")
sim = vpb.PyBulletSimulator(urdf_path, vamp.ROBOT_JOINTS['ur5'], True)

for cube in cubes:
    vamp_cube = np.array([transform_to_vamp_coords(point) for point in cube])
    center, euler, half_extents = calculate_cuboid(vamp_cube)
    print(f"Center: {center}, Euler: {euler}, Half extents: {half_extents}")
    sim.add_cuboid(half_extents, center, euler)

# config = robot.get_config()
# show the cuboid using vpb
sim.set_camera(position=[4, -4, -3], look_at=[0, 0, 0])
sim.animate([home, ref_point_config])

