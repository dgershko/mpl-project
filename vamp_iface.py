import vamp
import numpy as np
from collections.abc import Sequence
from utils import calculate_cuboid, transform_to_vamp_coords

class VampInterface:
    def __init__(self):
        self.robot_name = 'ur5'
        self.algorithm_name = 'rrtc'
        self.vamp_module, self.planner_func, self.plan_settings, self.simp_settings = (
            vamp.configure_robot_and_planner_with_kwargs(self.robot_name, self.algorithm_name)
        )
        self.env = vamp.Environment()

    def add_cuboid_obstacle(self, points):
        vamp_points = np.array([transform_to_vamp_coords(point) for point in points])
        center, euler, half_extents = calculate_cuboid(vamp_points)
        self.env.add_cuboid(vamp.Cuboid(center, euler, half_extents))
    
    def add_sphere_obstacle(self, center, radius):
        vamp_center = transform_to_vamp_coords(center)
        self.env.add_sphere(vamp.Sphere(vamp_center, radius))
    
    def plan(self, start_config: list[float], goal_config: list[float]):
        if len(start_config) != 6 or len(goal_config) != 6:
            raise ValueError(f"Invalid start or goal configuration size, expected 6, got {len(start_config)} and {len(goal_config)} respectively.")

        if not self.vamp_module.validate(start_config, self.env):
            raise ValueError(f"Invalid start configuration: {start_config}")
        if not self.vamp_module.validate(goal_config, self.env):
            raise ValueError(f"Invalid goal configuration: {goal_config}")
        
        result = self.planner_func(start_config, goal_config, self.env, self.plan_settings)
        simple = self.vamp_module.simplify(result.path, self.env, self.simp_settings)
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

    def validate_config(self, config):
        return self.vamp_module.validate(config, self.env)

    def solve_fk(self, config):
        return [[sphere.x, sphere.y, sphere.z] for sphere in vamp.ur5.fk(config)]
    
    def reset_env(self):
        self.env = vamp.Environment()

    def which_collides(self, config, obstacle_dict):
        """
        For cuboid obstacles defined using points in real coords
        """
        collides_map = {}
        for name, obstacle in obstacle_dict.items():
            tmp_env = vamp.Environment()
            vamp_points = np.array([transform_to_vamp_coords(point) for point in obstacle])
            center, euler, half_extents = calculate_cuboid(vamp_points)
            tmp_env.add_cuboid(vamp.Cuboid(center, euler, half_extents))
            if self.vamp_module.validate(config, tmp_env):
                collides_map[name] = False
            else:
                collides_map[name] = True
    
    def calibrate_z(self, config, calibration_obstacle):
        """
        Calibration obstacle must be a cuboid, defined using points in real coords
        Config is a configuration in which /robot body/ is just touching the obstacle, along the z axis
        Note! do not use end effector, as the vamp module uses a large end effector by default
        Returns: z offset to add to points to translate into vamp coords
        """
        def transform(point, z_offset):
            point = np.array(point) + np.array([0, 0, z_offset])
            point = np.array([point[1], -point[0], point[2]])
            return point
    
        initial_z = 1.5
        step_size = 0.01
        z = initial_z
        while True:
            tmp_env = vamp.Environment()
            vamp_points = np.array([transform(point) for point in calibration_obstacle])
            center, euler, half_extents = calculate_cuboid(vamp_points)
            tmp_env.add_cuboid(vamp.Cuboid(center, euler, half_extents))
            if not self.vamp_module.validate(config, tmp_env):
                return z # collision detected, critical z found
            z -= step_size
