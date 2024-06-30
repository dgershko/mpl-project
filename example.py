import numpy as np
import time

from vamp_iface import VampInterface
from rtde_iface import rtdeRobot
from env_consts import obstacle_blocks, target_pose, ref_point_config, home, cube_coords, named_blocks, ref_standing_block, calibration_config
from camrea import RSCamera
ip = "192.168.56.101" # sim IP
# ip = "192.168.0.10" # real robot IP



## Example of using vamp interface to render the environment
# =================================================================================================
def vamp_render_example():
    vamp_iface = VampInterface()
    robot = rtdeRobot(ip)
    for cube in obstacle_blocks:
        vamp_iface.add_cuboid_obstacle(cube)

    vamp_iface.render_env()
    robot.close()
# =================================================================================================


## Example of using vamp to plan a path and execute it
# =================================================================================================
def vamp_plan_and_execute_example():
    vamp_iface = VampInterface()
    robot = rtdeRobot(ip)
    robot.move_to_config(home)

    plan = vamp_iface.plan(home, ref_point_config)

    robot.execute_plan(plan, speed=1.7, acceleration=0.5)

    print(f"config: {robot.get_config()}")
    robot.close()
# =================================================================================================


## Example of using vamp interface to generate a plan and then render it
# =================================================================================================
def vamp_plan_and_render_example():
    vamp_iface = VampInterface()
    robot = rtdeRobot(ip)
    for cube in obstacle_blocks:
        vamp_iface.add_cuboid_obstacle(cube)

    target_config = robot.get_IK(target_pose, home)

    plan = vamp_iface.plan(home, target_config)
    vamp_iface.render_plan(plan)
    robot.close()
# =================================================================================================


## Example of planning times using vamp interface
# =================================================================================================
def vamp_planning_times_example():
    vamp_iface = VampInterface()
    robot = rtdeRobot(ip)
    robot.move_to_config(home)
    current_config = home
    target_config = robot.get_IK(target_pose, home)
    for cube in obstacle_blocks:
        vamp_iface.add_cuboid_obstacle(cube)

    planning_start = time.time_ns()
    plan = vamp_iface.plan(current_config, target_config)
    planning_end = time.time_ns()
    print(f"Planning time: {(planning_end - planning_start) / 1e9}s")
    print(f"Plan length: {len(plan)}")

    robot.execute_plan(plan, speed=1.7, acceleration=0.5)

    print(f"target config: {target_config}, actual config: {robot.get_config()}")
    print(f"target pose: {target_pose}, actual pose: {robot.get_pose()}")
    print(f"Avg. difference in pose: {np.mean(np.abs(np.array(target_pose) - np.array(robot.get_pose())))}")
    robot.close()
# =================================================================================================


## Example of various functions in rtde_iface
# =================================================================================================
def rtde_iface_example():
    robot = rtdeRobot(ip)

    robot.move_to_config(home)

    # robot status functions
    print(f"config: {robot.get_config()}")
    print(f"pose: {robot.get_pose()}")
    print(f"FK for robot in current pose: {robot.get_FK(robot.get_config())}")

    # ik functions
    print(f"target pose: {target_pose}")
    print(f"ik solution of target pose: {robot.get_IK(target_pose)}")
    print(f"ik solution of current pose which is closest to home: {robot.get_IK(target_pose, home)}")
    print(f"ik solution of current pose which is closest to home within 10cm of position, orientation: {robot.get_IK(target_pose, home, max_position_error=0.1, max_orientation_error=0.1)}")
    print(f"robot is running: {robot.is_program_running()}")

    # robot move functions
    permuted_home = home + np.random.rand(6) * 0.4
    robot.move_to_config(permuted_home)
    robot.move_to_config(home, speed=1.7, acceleration=0.5) # can set speed and accel
    print("starting async move")
    robot.move_to_config(permuted_home, is_async=True) # move in async mode
    counter = 0
    while not np.allclose(robot.get_config(), permuted_home, atol=1e-2):
        time.sleep(0.05)
        counter += 1
        pass
    print(f"async move done, {counter} iterations")

    robot.move_to_config(home, is_async=True)
    robot.stop()

    plan = [home, permuted_home] * 5
    robot.execute_plan(plan, speed=1.7, acceleration=0.5) # can execute a plan which consists of a series of moves

    # misc. functions
    print(f"program running: {robot.is_program_running()}")
    print(f"pose validation: {robot.validate_pose(target_pose)}")
    print(f"config validation: {robot.validate_config(home)}")
    print("starting freedrive mode")
    robot.start_freedrive_mode() # start freedrive mode
    time.sleep(5)
    robot.end_freedrive_mode() # end freedrive mode
    print("freedrive mode ended")
    robot.close()
# =================================================================================================


## Example of various functions in vamp_iface
# =================================================================================================
def vamp_iface_example():
    vamp_iface = VampInterface()

    # adding cuboid obstacles
    for cube in obstacle_blocks:
        vamp_iface.add_cuboid_obstacle(cube)

    # adding spherical obstacles
    for x, y, z in cube_coords:
        vamp_iface.add_sphere_obstacle([x, z, y], 0.025)

    # config validation, collision checking
    print(f"config validation: [legal] {vamp_iface.validate_config(home)}")
    print(f"config validation: [illegal] {vamp_iface.validate_config(ref_point_config)}")
    print(f"colliding obstacles: {vamp_iface.which_collides(ref_point_config, named_blocks)}")

    # planning
    robot.close()
    robot = rtdeRobot(ip)
    start_time = time.time_ns()
    plan = vamp_iface.plan(home, robot.get_IK(target_pose, home))
    end_time = time.time_ns()
    print(f"Plan length: {len(plan)}, took {(end_time - start_time) / 1e9}s to plan")

    vamp_iface.reset_env() # remove obstacles from environment
    start_time = time.time_ns()
    plan = vamp_iface.plan(home, ref_point_config)
    end_time = time.time_ns()
    print(f"Plan length: {len(plan)}, took {(end_time - start_time) / 1e9}s to plan")


    for cube in obstacle_blocks:
        vamp_iface.add_cuboid_obstacle(cube)

    # adding spherical obstacles
    for x, y, z in cube_coords:
        vamp_iface.add_sphere_obstacle([x, z, y], 0.025)
    random_config = vamp_iface.generate_random_config()
    print(f"random config: {random_config}")
    print(f"config valid: {vamp_iface.validate_config(random_config)}")
    time_start_ns = time.time_ns()
    plan = vamp_iface.plan(home, random_config)
    time_end_ns = time.time_ns()
    print(f"Plan length: {len(plan)}, took {(time_end_ns - time_start_ns) / 1e9}s to plan")

    robot.close()
# =================================================================================================


## Example of calibrating z offset
# =================================================================================================
def calibrate_z():
    robot = rtdeRobot(ip)
    vamp_iface = VampInterface()
    robot.start_freedrive_mode()
    print("Move robot to position where it's body touches the reference obstacle")
    input("Press enter when robot is in position")
    robot.end_freedrive_mode()
    reference_obstacle = ref_standing_block
    calibration_config = robot.get_config()
    z_offset = vamp_iface.calibrate_z(calibration_config, reference_obstacle)
    print(f"Calibrated z offset: {z_offset}")
    print(f"modify the z offset in env_consts.py")
    robot.close()
# =================================================================================================

## Example of dynamic obstacle planning
# Note! this requires implementing a way to get dynamic obstacles from the camera
# =================================================================================================
def dynamic_obstacle_planning():
    robot = rtdeRobot(ip)
    vamp_iface = VampInterface()
    camera = RSCamera()

    robot.start_freedrive_mode()
    input()
    robot.end_freedrive_mode()

    start_config = home
    target_config = robot.get_IK(target_pose, home)

    for cube in obstacle_blocks:
        vamp_iface.add_cuboid_obstacle(cube)

    plan = vamp_iface.plan(start_config, target_config)
    robot.execute_plan(plan, speed=1.7, acceleration=0.5, is_async=True)
    while not np.allclose(robot.get_config(), target_config, atol=1e-2):
        vamp_iface.reset_dynamic_obstacles()
        dynamic_obstacles: list[list[float]] = camera.get_obstacles()
        vamp_iface.set_dynamic_obstacles(dynamic_obstacles)
        if not vamp_iface.validate_plan(plan):
            print("Plan is invalid, replanning")
            robot.stop()
            plan = vamp_iface.plan(robot.get_config(), target_config)
            robot.execute_plan(plan, speed=1.7, acceleration=0.5, is_async=True)
    robot.close()
# =================================================================================================

if __name__ == "__main__":
    robot = rtdeRobot("192.168.0.10")
    camera = RSCamera()
    # robot.start_freedrive_mode()
    # print("started freedrive")
    # input()
    # robot.end_freedrive_mode()
    # exit()
    pose = robot.get_pose()
    while True:
        try:
            print(camera.get_obstacles(pose))
            input()
        except KeyboardInterrupt:
            break
    # camera.watch(pose)
    exit()
    time_start = time.time()
    counter = 0
    while True:
        try:
            counter += 1
            obstacles = camera.get_obstacles(pose)
            print(counter, obstacles, end = "\r")
        except KeyboardInterrupt:
            break
    time_end = time.time()
    print(f"time taken: {(time_end - time_start) / counter}")
    print(obstacles)
    robot.close()
    # from env_consts import big_standing_block
    # print(robot.get_config())
    # vamp_render_example()
    # vamp_plan_and_execute_example()
    # vamp_plan_and_render_example()
    # vamp_planning_times_example()
    # rtde_iface_example()
    # vamp_iface_example()
    # calibrate_z()
    # dynamic_obstacle_planning()
    # robot.close()
    pass