import urx
import numpy as np
from urx import RobotException
import multiprocessing
from time import sleep, time
from urx import Pose

class UrxIface():
    """
    Class to interface with the robot using URX.
    As the URX library is running in listener mode, it is run in a different proccess.
    """
    def __init__(self, ip="192.168.56.101"):
        self._create_urx_proccess(ip)
            
    def set_speed(self, speed: float, acceleration: float):
        self.parent_conn.send("set_speed")
        self.parent_conn.send((speed, acceleration))

    def get_config(self):
        self.parent_conn.send("get_config")
        return self.parent_conn.recv()

    def move_to_config(self, config: np.ndarray):
        if len(config) != 6:
            return "invalid config size"
        if not all(-np.pi <= angle <= np.pi for angle in config):
            return "invalid config values"
        self.parent_conn.send("move_to_config")
        self.parent_conn.send(config)
        result = self.parent_conn.recv()
        if not result == "success":
            raise RobotException(f"Failed to move to config: {config}")
    
    def execute_plan(self, plan: list[np.ndarray]):
        if any([len(config) != 6 for config in plan]):
            return "invalid configs detected"
        if not all([all(-np.pi <= angle <= np.pi for angle in config) for config in plan]):
            return "Invalid config values"
        
        self.parent_conn.send("execute_plan")
        self.parent_conn.send(plan)
        result = self.parent_conn.recv()
        if not result == "success":
            raise RobotException(f"fuck")
    
    def get_pos(self):
        self.parent_conn.send("get_pos")
        res = self.parent_conn.recv()
        while not isinstance(res, np.ndarray):
            res = self.parent_conn.recv()
        return res
    
    def close(self):
        self.parent_conn.send("close")
        self.listener.join()
    
    def __del__(self):
        if self.listener.is_alive():
            self.close()
    
    def _recv_with_timeout(self, timeout: float = 1):
        start_time = time()
        while not self.parent_conn.poll(1) and time() - start_time < timeout:
            sleep(0.1)
        if not self.parent_conn.poll():
            return None
        return self.parent_conn.recv()
    
    def _create_urx_proccess(self, ip: str, timeout: float = 2, num_attempts: int = 5):
        for attempt in range(num_attempts):
            self.parent_conn, self.child_conn = multiprocessing.Pipe()
            self.listener = multiprocessing.Process(target=urx_listener, args=(ip, self.child_conn))
            self.listener.start()

            if self.parent_conn.poll(timeout):
                response = self.parent_conn.recv()
                if response == "ready":
                    print(f"Listener started successfully on attempt {attempt + 1}")
                    return True
                else:
                    print(f"Listener process error on attempt {attempt + 1}: {response}")
            else:
                print(f"Timeout waiting for listener process on attempt {attempt + 1}")

            self.listener.terminate()
            self.listener.join()
            sleep(0.5)

        print("Failed to start listener process after maximum attempts")
        return False

        

def attempt_connection(ip: str):
    robot = None
    tries = 0
    while not robot:
        try:
            robot = urx.Robot(ip)
        except Exception as e:
            robot = None
            tries += 1
            if tries > 5:
                return False
            sleep(0.2)
    return robot

def urx_listener(ip: str, pipe: multiprocessing.Pipe):
    """
    Function to run in a separate process to listen for commands from the main process
    """
    print(f"Starting listener for {ip}")
    robot = attempt_connection(ip)
    if not robot:
        pipe.send("failed")
        return
    pipe.send("ready")
    speed = 0.5
    acceleration = 0.5
    while True:
        try:
            command = pipe.recv()
            match command:
                case "get_config":
                    pipe.send(robot.getj())
                case "move_to_config":
                    config = pipe.recv()
                    robot.movej(config, wait=False, acc=acceleration, vel=speed)
                    sleep(0.2)
                    while robot.is_program_running():
                        sleep(0.1)
                    if np.allclose(robot.getj(), config, atol=0.1):
                        pipe.send("success")
                    else:
                        pipe.send("failed")
                case "execute_plan":
                    plan = pipe.recv()
                    # robot.movep(plan, wait=True)
                    robot.movels(plan)
                    sleep(0.5)
                    while robot.is_program_running():
                        sleep(0.1)
                    if np.allclose(robot.getj(), plan[-1], atol=0.1):
                        pipe.send("success")
                    else:
                        pipe.send("failed")
                case "set_speed":
                    speed, acceleration = pipe.recv()
                    pipe.send("success")
                case "get_pos":
                    pipe.send(robot.get_pos().get_array())
                case "close":
                    robot.close()
                    break
                case _:
                    pipe.send("invalid command")
        except RobotException as e:
            robot.close()
            robot = attempt_connection(ip)
            if not robot:
                pipe.send("failed")
                return