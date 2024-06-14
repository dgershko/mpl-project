from copy import copy
import numpy as np
from pathlib import Path
import lab3_consts as consts

from vamp_iface import VampInterface
from urx_iface import UrxIface


static_obstacles = np.load("obstacles.npy")
ik_solutions = np.load("ik_solutions.npy")

home = consts.home
cube_coords = consts.cube_coords
init_coords = cube_coords.copy()
cube_approaches = consts.cube_approaches
cubes_actual = consts.cubes_actual

vamp_iface = VampInterface()
urx_iface = UrxIface()

def get_cube_obstacles(cube_coords):
    cube_obstacles = []
    for cube_coord in cube_coords:
        cube_obstacles.append(np.append(cube_coord, 0.025))
    return cube_obstacles


cube_obstacles = get_cube_obstacles(init_coords)
obstacles = np.vstack((cube_obstacles.copy(), static_obstacles))
urx_iface.move_to_config(home)
current_position = home
for i, cube_approach in enumerate(cube_approaches):
    cube_obstacles = get_cube_obstacles(cube_coords)
    # approach cube
    obstacles = np.vstack((cube_obstacles.copy(), static_obstacles))
    plan = vamp_iface.plan(current_position, cube_approach, obstacles)
    for configuration in plan:
        urx_iface.move_to_config(configuration)
    current_position = cube_approach
    # grab cube
    obstacles = np.vstack((get_cube_obstacles(cube_coords[:i] + cube_coords[i+1:]), static_obstacles))
    plan = vamp_iface.plan(current_position, cubes_actual[i], obstacles)
    for configuration in plan:
        urx_iface.move_to_config(configuration)
    current_position = cubes_actual[i]
    # lift cube
    plan = vamp_iface.plan(current_position, cube_approach, obstacles)
    for configuration in plan:
        urx_iface.move_to_config(configuration)

    current_position = cube_approach
    # place cube
    near_config = ik_solutions[i]
    plan = vamp_iface.plan(current_position, near_config, obstacles)
    for configuration in plan:
        urx_iface.move_to_config(configuration)

    current_position = near_config
    # plan_waypoints_with_obstacles.append([near_config, obstacles.copy()])
    # update cube position to target position
    cube_coords[i] = consts.dgershko_cubes[i]

obstacles = get_cube_obstacles(cube_coords) + static_obstacles
# plan_waypoints_with_obstacles.append([home, obstacles]) # return to home
# plan.extend(vamp_iface.plan(current_position, home, obstacles))

# for configuration in plan:
#     print(configuration)
# exit()

# import vamp
# from vamp import pybullet_interface as vpb
# robot_dir = Path(__file__).parent.parent / 'resources' / 'ur5'
# sim = vpb.PyBulletSimulator(str(robot_dir / f"ur5_spherized.urdf"), vamp.ROBOT_JOINTS['ur5'], True)
# for obstacle in obstacles:
#     sim.add_sphere(obstacle[1], obstacle[0])
# sim.animate(plan)

# for (waypoint, obstacles) in plan_waypoints_with_obstacles:
#     print(waypoint)
#     for obstacle in obstacles:
#         print("\t", obstacle)

