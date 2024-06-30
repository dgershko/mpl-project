# mpl-project

This is a project for the Technion's Motion Planning Lab.


# Contents
1. Python interface for Kavraki Lab's VAMP solver, contained in vamp_iface.py
2. Python interface for URX package for controlling UR robots, contained in urx_iface.py
3. Python interface for the RTDE package for controlling UR robots, contained in rtde_iface.py
4. Dockerfile definition for running a simulation of the UR robot in a container
5. Example of using these interfaces for solving a motion planning problem in real time in main.py


# Installation
1. Clone the [vamp repository](https://github.com/KavrakiLab/vamp)
2. Build and install the vamp python package by running the following commands:
```cd vamp && pip install .```
3. [If using the ur_iface] Install the URX python package by running ```pip install urx```
4. [If using the rtde_iface, recommended] Install the RTDE python package by running ```pip install ur_rtde```
5. To use the depth camera, install the cv2 and pyrealsense2 packages by running ```pip install opencv-python pyrealsense2``` (note, this requires python3.11 or **lower**)

# Containerization
There's already an existing Docker image for running the robot's simulation. 
However, it requires some set up to work properly, so for this project we created a Dockerfile which builds on top of the existing image and adds the necessary files and configurations to run the simulation.
To build the image, run the following command:
```bash
docker build -t ur5-sim .
```
To run this image as a container:
```bash
docker network create --subnet=192.168.56.0/24 ursim_net # run this once!
docker run \
  --rm \
  -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -p 30000-30100:30000-30100 \
  -p 50000-50100:50000-50100 \
  --net ursim_net \
  --ip 192.168.56.101 \
  ur5-sim
```
The simulation will be available through vnc at `localhost:5900` and through a web browser at `http://192.168.56.101:6080/vnc.html?host=192.168.56.101&port=6080`

# VampInterface Class Documentation

## Overview
The `VampInterface` class serves as a wrapper around the VAMP robotics planning and simulation library. It provides methods to configure the environment with obstacles, generate and validate motion plans, and visualize the environment and plans using a PyBullet simulation.

## Initialization
- `__init__(self)`: Initializes the VampInterface instance with a specified robot and algorithm for planning. It also initializes an empty environment and lists for cuboid and sphere obstacles.

## Methods

### Obstacle Management
- `add_cuboid_obstacle(self, points)`: Adds a cuboid obstacle to the environment. `points` is a list of points defining the cuboid in real-world coordinates.
- `add_sphere_obstacle(self, center, radius)`: Adds a spherical obstacle to the environment. `center` is the center of the sphere in real-world coordinates, and `radius` is the sphere's radius.
- `set_dynamic_obstacles(self, dynamic_obstacles)`: Sets dynamic cuboid obstacles in the environment. `dynamic_obstacles` is a list of lists, where each inner list defines a cuboid obstacle's real world points.
- `reset_dynamic_obstacles(self)`: Resets the environment to only include the static obstacles that were added before.

### Planning and Validation
- `plan(self, start_config, goal_config)`: Generates a motion plan from a start configuration to a goal configuration. Both configurations must be lists of 6 floats representing joint positions.
- `validate_plan(self, plan)`: Validates a given plan by checking each configuration in the plan for collisions in the environment.
- `generate_random_config(self)`: Generates a random, valid configuration in the current environment.
- `validate_config(self, config)`: Validates a single configuration for collisions in the environment.
- `solve_fk(self, config)`: Solves the forward kinematics for a given configuration and returns the positions of the end effector.

### Environment and Visualization
- `reset_env(self)`: Resets the environment to an empty state, removing all obstacles.
- `render_env(self, config=home)`: Renders the current environment with an optional robot configuration using PyBullet.
- `render_plan(self, plan)`: Visualizes a given motion plan in the environment using PyBullet.

### Utility Methods
- `which_collides(self, config, obstacle_dict)`: Checks which obstacles from a given dictionary collide with a specified configuration.
- `calibrate_z(self, config, calibration_obstacle)`: Calibrates the Z offset for translating real-world coordinates to VAMP coordinates based on a calibration obstacle.

## Internal Methods
- `_transform_to_vamp_coords(point)`: Transforms a point from real-world coordinates to VAMP coordinates.
- `_get_cuboid_transform(points)`: Calculates the center, Euler angles, and half extents of a cuboid defined by its corner points in real-world coordinates.




# rtdeRobot Class Documentation

## Overview
The `rtdeRobot` class provides an interface to control and interact with a robot using the Real-Time Data Exchange (RTDE) protocol. It encapsulates functionalities for moving the robot, getting its state, and handling connection issues through automatic reconnection.

## Initialization
- `__init__(self, ip:str = "192.168.0.11")`: Initializes the `rtdeRobot` instance. Connects to the robot using RTDE control and receive interfaces at the specified IP address.

## Methods

### Connection Management
- `_reconnect(self)`: Attempts to reconnect both the RTDE control and receive interfaces. This method is called automatically upon encountering an error during any RTDE operation.

### Robot Movement and Configuration
- `get_config(self)`: Returns the current joint configuration of the robot.
- `move_to_config(self, config: list[float], speed: float = 1, acceleration: float = 1.4, is_async: bool = False)`: Moves the robot to the specified joint configuration (`config`) at the given `speed` and `acceleration`. Can be executed asynchronously if `is_async` is True.
- `execute_plan(self, plan: list[list[float]], speed: float = 1, acceleration: float = 1.4, blend: float = 0.01, is_async: bool = False)`: Executes a sequence of movements (`plan`) with specified `speed`, `acceleration`, and `blend` parameters. Optionally asynchronous.
- `stop(self, is_async: bool = False)`: Stops the robot's movement. Can be executed asynchronously if `is_async` is True.

### Kinematics
- `get_IK(self, tool_pose: list[float], q_near: list[float] = [], max_position_error: float = 0.01, max_orientation_error: float = 0.1) -> list[float]`: Calculates the inverse kinematics for a given tool pose. Returns the joint configuration that achieves the pose, if possible.
- `ik_exists(self, tool_pose: list[float], q_near: list[float] = [], max_position_error: float = 0.01, max_orientation_error: float = 0.1) -> bool`: Checks if a valid inverse kinematics solution exists for the given tool pose.
- `get_FK(self, config: list[float] = [])`: Calculates the forward kinematics for a given joint configuration. Returns the tool pose achieved by this configuration.

### State Queries
- `get_pose(self)`: Returns the current pose of the tool center point (TCP).
- `is_program_running(self) -> bool`: Checks if a program is currently running on the robot.
- `validate_pose(self, pose: list[float]) -> bool`: Validates if a given pose is within the robot's safety limits.
- `validate_config(self, config: list[float]) -> bool`: Validates if a given joint configuration is within the robot's safety limits.

### Mode Management
- `start_freedrive_mode(self)`: Enables freedrive mode, allowing the robot to be moved manually.
- `end_freedrive_mode(self)`: Disables freedrive mode.

### Cleanup
- `close(self)`: Safely disconnects the RTDE control and receive interfaces.
- `__del__(self)`: Destructor that ensures the connections are closed when the instance is deleted.

## Decorators
- `rtde_retry_decorator`: A decorator applied to most methods of the class. It automatically retries the operation once if an exception is caught, attempting to reconnect before retrying.


# UrxIface Class Documentation

## Overview
The `UrxIface` class provides an interface for interacting with a robot using the URX library. It operates in listener mode and runs in a separate process to handle commands such as moving the robot, setting speed, and getting the robot's configuration.

## Initialization
- `__init__(self, ip="192.168.56.101")`: Initializes the `UrxIface` instance and creates a URX process for communication with the robot at the specified IP address.

## Methods

### Movement and Configuration
- `set_speed(self, speed: float, acceleration: float)`: Sets the movement speed and acceleration of the robot.
- `get_config(self)`: Retrieves the current joint configuration of the robot.
- `move_to_config(self, config: np.ndarray)`: Moves the robot to the specified joint configuration. Validates the configuration size and value ranges before sending the command.
- `execute_plan(self, plan: list[np.ndarray])`: Executes a sequence of movements (plan) for the robot. Validates the configurations in the plan before execution.

### Position Queries
- `get_pos(self)`: Returns the current position of the robot as a NumPy array.

### Connection Management
- `close(self)`: Closes the connection to the robot and terminates the listener process.
- `__del__(self)`: Destructor method that ensures the listener process is closed if still alive.

### Internal Methods
- `_recv_with_timeout(self, timeout: float = 1)`: Receives data from the URX process with a specified timeout. Returns `None` if the timeout is exceeded.
- `_create_urx_proccess(self, ip: str, timeout: float = 2, num_attempts: int = 5)`: Attempts to create and start the URX process for a specified number of attempts. Returns `True` if successful, `False` otherwise.

## Auxiliary Functions
- `attempt_connection(ip: str)`: Attempts to establish a connection to the robot using the URX library. Retries up to 5 times before giving up.
- `urx_listener(ip: str, pipe: multiprocessing.Pipe)`: Function designed to run in a separate process. It listens for commands from the main process and interacts with the robot accordingly. Handles various commands such as getting the robot's configuration, moving to a configuration, executing a plan, setting speed, and closing the connection.

## Exceptions
- Raises `RobotException` if a movement to a specified configuration or execution of a plan fails.