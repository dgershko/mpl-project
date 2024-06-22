# mpl-project

This is a project for the Technion's Motion Planning Lab.


### Contents
1. Python interface for Kavraki Lab's VAMP solver, contained in vamp_iface.py
2. Python interface for URX package for controlling UR robots, contained in urx_iface.py
3. Python interface for the RTDE package for controlling UR robots, contained in rtde_iface.py
4. Dockerfile definition for running a simulation of the UR robot in a container
5. Example of using these interfaces for solving a motion planning problem in real time in main.py


### Installation
---
1. Clone the [vamp repository](https://github.com/KavrakiLab/vamp)
2. Build and install the vamp python package by running the following commands:
```bash
cd vamp
pip install .
```
3. Install the urx python package by running ```pip install urx```
4. Install the RTDE python package by running ```pip install ur_rtde```

### Containerization
---
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

### Usage
---
#### Vamp interface
Initialization

    __init__()
        Initializes the interface for the UR5 robot using the RRTC algorithm.

Methods

    plan(start_config: list[float], goal_config: list[float], obstacles: np.ndarray = []) -> list[list[float]]
        Plans a path from start_config to goal_config while avoiding specified obstacles.
        Parameters:
            start_config: List of 6 joint angles representing the start configuration.
            goal_config: List of 6 joint angles representing the goal configuration.
            obstacles: Optional NumPy array of obstacles, where each obstacle is defined by a center (x, y, z) and a radius.
        Returns: A list of configurations representing the planned path.

    generate_random_config(obstacles: list[list[float], float] = []) -> list[float]
        Generates a random valid configuration for the robot, avoiding specified obstacles.
        Parameters:
            obstacles: Optional list of obstacles, where each obstacle is defined by a center (x, y, z) and a radius.
        Returns: A list of 6 joint angles representing a random valid configuration.

    solve_fk(config: list[float]) -> list[list[float]]
        Solves the forward kinematics for a given robot configuration.
        Parameters:
            config: List of 6 joint angles representing the robot configuration.
        Returns: A list of positions [x, y, z] for each sphere in the forward kinematics result.

#### RTDE interface:
Initialization

    __init__(ip: str = "192.168.0.11")
        Initializes the robot interface with the given IP address.

Methods

    get_config() -> list[float]
        Retrieves the current joint configuration of the robot.

    move_to_config(config: list[float], speed: float = 1, acceleration: float = 1.4, is_async: bool = False)
        Moves the robot to the specified joint configuration with the given speed and acceleration.

    execute_plan(plan: list[list[float]], speed: float = 1, acceleration: float = 1.4, blend: float = 0.03, is_async: bool = False)
        Executes a planned sequence of joint configurations with the given speed, acceleration, and blend.

    stop(is_async: bool = False)
        Stops the robot's current motion.

    get_IK(tool_pose: list[float], q_near: list[float] = [], max_position_error: float = 0.01, max_orientation_error: float = 0.1) -> list[float]
        Computes the inverse kinematics for the given tool pose and optional nearby joint configuration.

    ik_exists(tool_pose: list[float], q_near: list[float] = [], max_position_error: float = 0.01, max_orientation_error: float = 0.1) -> bool
        Checks if a valid inverse kinematics solution exists for the given tool pose and optional nearby joint configuration.

    get_FK(config: list[float] = []) -> any
        Computes the forward kinematics for the given joint configuration. If not provided, the current joint configuration is used.

    get_pose() -> list[float]
        Retrieves the current pose of the robot's tool.

    is_program_running() -> bool
        Checks if a program is currently running on the robot.

    validate_pose(pose: list[float]) -> bool
        Validates if the given pose is within the robot's safety limits.

    validate_config(config: list[float]) -> bool
        Validates if the given joint configuration is within the robot's safety limits.

    start_freedrive_mode()
        Starts the freedrive mode, allowing manual manipulation of the robot.

    end_freedrive_mode()
        Ends the freedrive mode.

    close()
        Closes the connection to the robot.


#### URX interface:
This interface is used to control the UR robot. 
It is an interface to the URX package, along with some quality of life improvements (such as automatic reconnection)
The interface is used to send commands to the robot, such as moving it to a specific position, or getting its current position.
It provides the following API:

Initialization

    __init__(ip: str = "192.168.56.101")
        Initializes the interface with the robot at the specified IP address.

Methods

    set_speed(speed: float, acceleration: float)
        Sets the speed and acceleration of the robot.

    get_config() -> np.ndarray
        Retrieves the current configuration (joint positions) of the robot.

    move_to_config(config: np.ndarray)
        Moves the robot to the specified configuration. The configuration should be a numpy array of length 6 with values in the range [-π, π].

    get_pos() -> np.ndarray
        Gets the current position of the robot's tool.

    close()
        Closes the connection to the robot and terminates the listener process.