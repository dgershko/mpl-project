from copy import copy
import numpy as np
from pathlib import Path

import lab3_consts as consts
from lab3_utils import get_current_obstacles, execute_plan, animate_plan, transform_obstacles_to_vamp, render_config, static_obstacles

from vamp_iface import VampInterface
from urx_iface import UrxIface
from rtde_iface import rtdeRobot

# static_obstacles = np.load("obstacles.npy")
ik_solutions = np.load("ik_solutions.npy")

home_config = consts.home
cube_coords = consts.cube_coords
initial_cube_coords = cube_coords.copy()
cube_approach_configurations = consts.cube_approaches
cube_actual_configurations = consts.cubes_actual

vamp_iface = VampInterface()
rtde_robot = rtdeRobot("192.168.56.101")


rtde_robot.move_to_config(home_config)
current_config = home_config
for i, cube_approach_config in enumerate(cube_approach_configurations):
    if i < 2:
        continue
    # approach cube
    obstacles = get_current_obstacles(cube_coords)
    vamp_obstacles = transform_obstacles_to_vamp(obstacles)
    try:
        plan = vamp_iface.plan(current_config, cube_approach_config, vamp_obstacles)
    except:
        render_config(cube_approach_config, obstacles)
        raise
    print(f"Moving to cube {i}")
    # rtde_robot.execute_plan(plan)
    rtde_robot.execute_plan(plan)
    current_config = cube_approach_config

    # grab cube
    obstacles = get_current_obstacles(cube_coords[:i] + cube_coords[i+1:])
    vamp_obstacles = transform_obstacles_to_vamp(obstacles)
    plan = [current_config, cube_actual_configurations[i]]
    print(f"Grabbing cube {i}")
    rtde_robot.execute_plan(plan)
    current_config = cube_actual_configurations[i]

    # lift cube
    plan = [current_config, cube_approach_config]
    print(f"Lifting cube {i}")
    rtde_robot.execute_plan(plan)
    current_config = cube_approach_config

    # place cube
    near_config = ik_solutions[i]
    plan = vamp_iface.plan(current_config, near_config, vamp_obstacles)
    print(f"Placing cube {i}")
    rtde_robot.execute_plan(plan)
    current_config = near_config

    cube_coords[i] = consts.dgershko_cubes[i] # update cube coords



