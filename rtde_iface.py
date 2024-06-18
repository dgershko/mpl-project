import rtde_control
import rtde_receive
import numpy as np

class rtdeRobot():
    def __init__(self, ip:str = "192.168.0.11"):
        self.control = rtde_control.RTDEControlInterface(ip)
        self.receive = rtde_receive.RTDEReceiveInterface(ip)
    
    def get_config(self):
        return self.receive.getActualQ()
    
    def move_to_config(self, config: list[float]):
        self.control.moveJ(config)
    
    def execute_plan(self, plan: list[list[float]], speed: float = 1, acceleration: float = 1.4, blend: float = 0.03):
        plan = np.array(plan)
        plan_with_params = np.array([np.concatenate([config, [speed, acceleration, blend]]) for config in plan])
        # plan = np.hstack((plan, np.repeat([speed, acceleration, blend], len(plan), axis=1)))
        # plan = [config + [speed, acceleration, blend] for config in plan]
        self.control.moveJ(plan_with_params)
    
    def close(self):
        try:
            self.control.disconnect()
            self.receive.disconnect()
        except:
            pass
    
    def __del__(self):
        self.close()