import numpy as np
from time import perf_counter

from rtde_iface import rtdeRobot
from vamp_iface import VampInterface
from lab3_consts import home
from env_consts import target_pose_1, target_pose_2, target_pose_3
from env_consts import obstacle_blocks, named_blocks, wall, floor, pose_list_1


poses = [target_pose_1, target_pose_2, target_pose_3]
robot = rtdeRobot("192.168.56.101")
vamp_iface = VampInterface()


vamp_iface.add_cuboid_obstacle(wall)
vamp_iface.add_cuboid_obstacle(floor)

current_config = home
for index, pose in enumerate(pose_list_1):
    print(f"Moving to pose {index + 1}")
    if not robot.ik_exists(pose, q_near=current_config):
        raise Exception("No IK exists!")
    ik_sol = robot.get_IK(pose, q_near=current_config)
    print("IK solution: ", ik_sol)
    planning_time_start = perf_counter()
    try:
        vamp_plan = vamp_iface.plan(current_config, ik_sol)
    except:
        robot.move_to_config(ik_sol)
        print(f"Failed to plan to pose {index + 1}")
        vamp_iface.which_collides(ik_sol, named_blocks)
        exit()
    planning_time_end = perf_counter()
    print(f"Planning time: {planning_time_end - planning_time_start}")
    """ if camera works:
    robot.execute_plan(vamp_plan, asynchronous=True)
    while robot.is_program_running():
        # get input from camera
        # replan if problems appear
        pass
    """
    robot.execute_plan(vamp_plan, speed=3, acceleration=2)
    
    current_config = ik_sol

# move back to home
start_config = robot.get_config()
plan = vamp_iface.plan(current_config, home)
robot.execute_plan(plan, speed=3, acceleration=1.5)