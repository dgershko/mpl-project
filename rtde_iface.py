import rtde_control
import rtde_receive
import functools
import numpy as np


def rtde_retry_decorator(method):
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        try:
            return method(self, *args, **kwargs)
        except:
            self._reconnect()
            return method(self, *args, **kwargs)
    return wrapper

class rtdeRobot():
    def __init__(self, ip:str = "192.168.0.11"):
        self.control = rtde_control.RTDEControlInterface(ip)
        self.receive = rtde_receive.RTDEReceiveInterface(ip)
    
    def _reconnect(self):
        self.control.reconnect()
        self.receive.reconnect()

    @rtde_retry_decorator
    def get_config(self):
        return self.receive.getActualQ()
    
    @rtde_retry_decorator
    def move_to_config(self, config: list[float], speed: float = 1, acceleration: float = 1.4, is_async: bool = False):
        self.control.moveJ(config, speed, acceleration, is_async)
    
    @rtde_retry_decorator
    def execute_plan(self, plan: list[list[float]], speed: float = 1, acceleration: float = 1.4, blend: float = 0.01, is_async: bool = False):
        plan = np.array(plan)
        plan_with_params = np.array([np.concatenate([config, [speed, acceleration, blend]]) for config in plan])
        self.control.moveJ(plan_with_params)
        self.move_to_config(plan[-1], speed, acceleration, is_async)
    
    @rtde_retry_decorator
    def stop(self, is_async: bool = False):
        self.control.stopJ(asynchronous=is_async)

    @rtde_retry_decorator
    def get_IK(self, tool_pose: list[float], q_near: list[float] = [], max_position_error: float = 0.01, max_orientation_error: float = 0.1) -> list[float]:
        if not self.ik_exists(tool_pose, q_near, max_position_error, max_orientation_error):
            return None
        """
        tool_pose: [x, y, z, rx, ry, rz] - pose of the tool
        q_near: [q1, q2, q3, q4, q5, q6] - nearby pose of the robot [OPTIONAL]
        max_position_error: float - maximum position error [OPTIONAL]
        max_orientation_error: float - maximum orientation error [OPTIONAL]
        """
        return self.control.getInverseKinematics(tool_pose, q_near, max_position_error, max_orientation_error)
    
    @rtde_retry_decorator
    def ik_exists(self, tool_pose: list[float], q_near: list[float] = [], max_position_error: float = 0.01, max_orientation_error: float = 0.1) -> bool:
        return self.control.getInverseKinematicsHasSolution(tool_pose, q_near, max_position_error, max_orientation_error)
    
    @rtde_retry_decorator
    def get_FK(self, config: list[float] = []):
        """
        config: [q1, q2, q3, q4, q5, q6] - joint configuration of the robot [OPTIONAL]
        If not provided, the current joint configuration is used
        returns transformation matrix? of something? idk
        """
        return self.control.getForwardKinematics(config)
    
    @rtde_retry_decorator
    def get_pose(self):
        return self.receive.getActualTCPPose()
    
    @rtde_retry_decorator
    def is_program_running(self) -> bool:
        return self.control.isProgramRunning()

    @rtde_retry_decorator
    def validate_pose(self, pose: list[float]) -> bool:
        return self.control.isPoseWithinSafetyLimits(pose)
    
    @rtde_retry_decorator
    def validate_config(self, config: list[float]) -> bool:
        return self.control.isJointsWithinSafetyLimits(config)
    
    @rtde_retry_decorator
    def start_freedrive_mode(self):
        self.control.freedriveMode()

    @rtde_retry_decorator
    def end_freedrive_mode(self):
        self.control.endFreedriveMode()

    def close(self):
        try:
            self.control.disconnect()
            self.receive.disconnect()
        except:
            pass
    
    def __del__(self):
        self.close()