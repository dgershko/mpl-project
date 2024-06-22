import rtde_control
import rtde_receive

import numpy as np

class rtdeRobot():
    def __init__(self, ip:str = "192.168.0.11"):
        self.control = rtde_control.RTDEControlInterface(ip)
        self.receive = rtde_receive.RTDEReceiveInterface(ip)
    
    def get_config(self):
        return self.receive.getActualQ()
    
    def move_to_config(self, config: list[float], speed: float = 1, acceleration: float = 1.4, is_async: bool = False):
        self.control.moveJ(config, speed, acceleration, is_async)
    
    def execute_plan(self, plan: list[list[float]], speed: float = 1, acceleration: float = 1.4, blend: float = 0.03, is_async: bool = False):
        plan = np.array(plan)
        plan_with_params = np.array([np.concatenate([config, [speed, acceleration, blend]]) for config in plan], is_async)
        self.control.moveJ(plan_with_params)
    
    def stop(self, is_async: bool = False):
        self.control.stopJ(asynchronous=is_async)

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
    
    def ik_exists(self, tool_pose: list[float], q_near: list[float] = [], max_position_error: float = 0.01, max_orientation_error: float = 0.1) -> bool:
        return self.control.getInverseKinematicsHasSolution(tool_pose, q_near, max_position_error, max_orientation_error)
    
    def get_FK(self, config: list[float] = []):
        """
        config: [q1, q2, q3, q4, q5, q6] - joint configuration of the robot [OPTIONAL]
        If not provided, the current joint configuration is used
        returns transformation matrix? of something? idk
        """
        return self.control.getForwardKinematics(config)
    
    def get_pose(self):
        return self.receive.getActualTCPPose()
    
    def is_program_running(self) -> bool:
        return self.control.isProgramRunning()

    def validate_pose(self, pose: list[float]) -> bool:
        return self.control.isPoseWithinSafetyLimits(pose)
    
    def validate_config(self, config: list[float]) -> bool:
        return self.control.isJointsWithinSafetyLimits(config)
    
    def start_freedrive_mode(self):
        self.control.freedriveMode()

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