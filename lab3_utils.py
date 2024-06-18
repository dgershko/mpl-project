# python imports
import numpy as np
import time
from pathlib import Path
import matplotlib.pyplot as plt

# library imports
import vamp
from vamp import pybullet_interface as vpb

# local imports
from urx_iface import UrxIface
from vamp_iface import VampInterface

static_obstacles = np.load("obstacles.npy")
# change size of each obstacle to 0.01
static_obstacles[:, -1] = 0.01


def get_current_obstacles(cube_coords) -> np.ndarray:
    obstacles = []
    for cube_coord in cube_coords:
        obstacles.append(np.append(cube_coord, 0.025))
    obstacles = np.vstack((obstacles, static_obstacles))
    return obstacles

def execute_plan(urx_iface: UrxIface, plan):
    start_time = time.time()
    for i, configuration in enumerate(plan):
        urx_iface.move_to_config(configuration)
        print(" " * 80, end="\r")
        print(f"Moving to config #{i} of {len(plan) - 1}", end="\r")
    end_time = time.time()
    total_time = end_time - start_time
    print(f"Plan execution time: {total_time:.2f}s, {total_time / len(plan):.2f}s per config")

def animate_plan(plan, obstacles):
    urdf_path = str(Path(__file__).parent / "ur5" / "ur5_spherized.urdf")
    print(urdf_path)
    sim = vpb.PyBulletSimulator(urdf_path, vamp.ROBOT_JOINTS['ur5'], True)
    for obstacle in obstacles:
        sim.add_sphere(obstacle[3], obstacle[:3])
    sim.animate(plan)

def transform_to_vamp_coords(point):
    # translate by -0.85 in z
    point = np.array(point) + np.array([0, 0, 0.85])
    # rotate 90 degrees clockwise around z axis
    point = np.array([point[1], -point[0], point[2]])
    return point

def transform_obstacles_to_vamp(obstacles):
    transformed_obstacles = []
    for obstacle in obstacles:
        transformed_coords = transform_to_vamp_coords(obstacle[:3])
        transformed_obstacles.append(np.append(transformed_coords, obstacle[3]))
    return np.array(transformed_obstacles)


def render_config(config, obstacles):
    vamp_obstacles = transform_obstacles_to_vamp(obstacles)
    # draw in plt
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    for sphere in vamp_obstacles:
        ax.scatter(*sphere[:3], c='r', s=sphere[3] * 1000, alpha=0.5)
    robot_spheres = [[sphere.x, sphere.y, sphere.z] for sphere in vamp.ur5.fk(config)]
    for sphere in robot_spheres:
        ax.scatter(*sphere[:3], c='b', s=25)
    for sphere in obstacles:
        ax.scatter(*sphere[:3], c='g', s=25)
    plt.show()
