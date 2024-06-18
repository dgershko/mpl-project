import vamp
import numpy as np
from collections.abc import Sequence

class VampInterface:
    def __init__(self):
        self.robot_name = 'ur5'
        self.algorithm_name = 'rrtc'
        self.vamp_module, self.planner_func, self.plan_settings, self.simp_settings = (
            vamp.configure_robot_and_planner_with_kwargs(self.robot_name, self.algorithm_name)
        )
    
    def plan(self, start_config: list[float], goal_config: list[float], obstacles: np.ndarray = []):
        if len(start_config) != 6 or len(goal_config) != 6:
            raise ValueError(f"Invalid start or goal configuration size, expected 6, got {len(start_config)} and {len(goal_config)} respectively.")

        env = vamp.Environment()
        for obstacle in obstacles:
            obstacle_center, obstacle_radius = obstacle[:3], obstacle[3]
            env.add_sphere(vamp.Sphere(obstacle_center, obstacle_radius))

        if not self.vamp_module.validate(start_config, env):
            raise ValueError(f"Invalid start configuration: {start_config}")
        if not self.vamp_module.validate(goal_config, env):
            raise ValueError(f"Invalid goal configuration: {goal_config}")
        
        result = self.planner_func(start_config, goal_config, env, self.plan_settings)
        simple = self.vamp_module.simplify(result.path, env, self.simp_settings)
        simple.path.interpolate(vamp.ur5.resolution())
        path_configs = [config.to_list() for config in simple.path]
        return path_configs
    
    def generate_random_config(self, obstacles: list[list[float], float] = []):
        env = vamp.Environment()
        for sphere in obstacles:
            sphere_center, sphere_radius = sphere[0], sphere[1]
            env.add_sphere(vamp.Sphere(sphere_center, sphere_radius))
        while True:
            random_config_vector = list(np.random.uniform(low=-np.pi, high=np.pi, size=(6,)))
            if vamp.ur5.validate(random_config_vector, env):
                return random_config_vector

    def solve_ik(self, position):
        solutions = vamp.ur5.get_inverse_kinematics_solutions(position[0], position[1], position[2])
        return [solution.to_list() for solution in solutions]
    
    def solve_fk(self, config):
        return [[sphere.x, sphere.y, sphere.z] for sphere in vamp.ur5.fk(config)]
