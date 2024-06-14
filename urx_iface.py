import urx
import numpy as np
from urx import RobotException
import multiprocessing
from time import sleep

class UrxIface():
    """
    Class to interface with the robot using URX.
    As the URX library is running in listener mode, it is run in a different proccess.
    """
    def __init__(self, ip="192.168.56.101"):
        pipe = multiprocessing.Pipe()
        self.pipe = pipe[0]
        self.listener = multiprocessing.Process(target=urx_listener, args=(ip, pipe[1]))
        self.listener.start()

    def set_speed(self, speed: float, acceleration: float):
        self.pipe.send("set_speed")
        self.pipe.send((speed, acceleration))

    def get_config(self):
        self.pipe.send("get_config")
        return self.pipe.recv()

    def move_to_config(self, config: np.ndarray):
        if len(config) != 6:
            return "invalid config size"
        if not all(-np.pi <= angle <= np.pi for angle in config):
            return "invalid config values"
        self.pipe.send("move_to_config")
        self.pipe.send(config)
        result = self.pipe.recv()
        if not result == "success":
            raise RobotException(f"Failed to move to config: {config}")
    
    def close(self):
        self.pipe.send("close")
        self.listener.join()
    
    def __del__(self):
        if self.listener.is_alive():
            self.close()


def urx_listener(ip: str, pipe: multiprocessing.Pipe):
    """
    Function to run in a separate process to listen for commands from the main process
    """
    robot = urx.Robot(ip)
    speed = 0.5
    acceleration = 0.5
    try:
        while True:
            command = pipe.recv()
            match command:
                case "get_config":
                    pipe.send(robot.getj())
                case "move_to_config":
                    config = pipe.recv()
                    robot.movej(config, wait=False, acc=acceleration, vel=speed)
                    sleep(0.5)
                    while robot.is_program_running():
                        sleep(0.1)
                    status = "success" if np.allclose(robot.getj(), config, atol=0.01) else "failed"
                    pipe.send(status)
                case "set_speed":
                    speed, acceleration = pipe.recv()
                    pipe.send("success")
                case "close":
                    robot.close()
                    break
                case _:
                    pipe.send("invalid command")
    except Exception as e:
        pipe.send(f"Listener error: {e}")
    finally:
        robot.close()
