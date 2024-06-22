import numpy as np
from time import perf_counter

from rtde_iface import rtdeRobot
from vamp_iface import VampInterface
from lab3_consts import home


plan_points = np.load("plan.npy") # list of robot poses - [x, y, z, rx, ry, rz]
robot = rtdeRobot("192.168.156.101")
vamp_iface = VampInterface()

robot.move_to_config(home)

# add obstacles

current_config = home
for point in plan_points:
    if not robot.ik_exists(point, q_near=current_config):
        raise Exception("No IK exists!")
    ik_sol = robot.get_IK(point, q_near=current_config)
    planning_time_start = perf_counter()
    vamp_plan = vamp_iface.plan(current_config, ik_sol) # add obstacles!!!
    planning_time_end = perf_counter()
    print(f"Planning time: {planning_time_end - planning_time_start}")
    """
    robot.execute_plan(vamp_plan, asynchronous=True)
    while robot.is_program_running():
        # get input from camera
        # replan if problems appear
        pass
    """
    robot.execute_plan(vamp_plan)
    current_config = ik_sol
