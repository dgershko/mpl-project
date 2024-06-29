import vamp
import numpy as np
from collections.abc import Sequence
from env_consts import home, z_offset
from concurrent.futures import ProcessPoolExecutor


def _transform_to_vamp_coords(point):
    # translate by -0.85 in z
    point = np.array(point) + np.array([-0.01, 0.02, z_offset])
    # rotate 90 degrees clockwise around z axis
    point = np.array([point[1], -point[0], point[2]])
    return point

def _get_cuboid_transform(points):
    vamp_points = np.array([_transform_to_vamp_coords(point) for point in points])
    center = np.mean(vamp_points, axis=0)
    half_extents = np.max(np.abs(vamp_points - center), axis=0)
    euler = [0, 0, 0]
    return center, euler, half_extents


class VampInterface:
    def __init__(self):
        self.robot_name = 'ur5'
        self.algorithm_name = 'rrtc'
        self.vamp_module, self.planner_func, self.plan_settings, self.simp_settings = (
            vamp.configure_robot_and_planner_with_kwargs(self.robot_name, self.algorithm_name)
        )
        self.cube_obstacles = []
        self.sphere_obstacles = []
        self.env = vamp.Environment()

    def add_cuboid_obstacle(self, points):
        center, euler, half_extents = _get_cuboid_transform(points)
        self.cube_obstacles.append([center, euler, half_extents])
        self.env.add_cuboid(vamp.Cuboid(center, euler, half_extents))
    
    def add_sphere_obstacle(self, center, radius):
        vamp_center = _transform_to_vamp_coords(center)
        self.sphere_obstacles.append([vamp_center, radius])
        self.env.add_sphere(vamp.Sphere(vamp_center, radius))
    
    def set_dynamic_obstacles(self, dynamic_obstacles: list[list[float]]):
        for obstacle in dynamic_obstacles:
            center, euler, half_extents = _get_cuboid_transform(obstacle)
            self.env.add_cuboid(vamp.Cuboid(center, euler, half_extents))
        
    def reset_dynamic_obstacles(self):
        self.env = vamp.Environment()
        for center, euler, half_extents in self.cube_obstacles:
            self.env.add_cuboid(vamp.Cuboid(center, euler, half_extents))
        for center, radius in self.sphere_obstacles:
            self.env.add_sphere(vamp.Sphere(center, radius))

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

    def validate_plan(self, plan):
        for config in plan:
            if not self.vamp_module.validate(config, self.env):
                return False
        return True
    
    def generate_random_config(self):
        """
        Generates a random configuration that is valid in the current environment
        """
        while True:
            random_config_vector = np.random.uniform(low=-np.pi, high=np.pi, size=(6,))
            if self.vamp_module.validate(random_config_vector, self.env):
                return random_config_vector

    def validate_config(self, config):
        return self.vamp_module.validate(config, self.env)

    def solve_fk(self, config):
        return [[sphere.x, sphere.y, sphere.z] for sphere in vamp.ur5.fk(config)]
    
    def reset_env(self):
        self.env = vamp.Environment()
        self.cube_obstacles = []
        self.sphere_obstacles = []

    def render_env(self, config=home):
        import vamp.pybullet_interface as vpb
        from pathlib import Path
        urdf_path = str(Path(__file__).parent / "ur5" / "ur5_spherized.urdf")
        sim = vpb.PyBulletSimulator(urdf_path, vamp.ROBOT_JOINTS['ur5'], True)
        for center, euler, half_extents in self.cube_obstacles:
            sim.add_cuboid(half_extents, center, euler)
        for center, radius in self.sphere_obstacles:
            sim.add_sphere(radius, center)
        sim.set_joint_positions(config)
        sim.set_camera(position=[2, -3, -2], look_at=[0, 0, 0])
        sim.animate([config])

    def render_plan(self, plan):
        import vamp.pybullet_interface as vpb
        from pathlib import Path
        urdf_path = str(Path(__file__).parent / "ur5" / "ur5_spherized.urdf")
        sim = vpb.PyBulletSimulator(urdf_path, vamp.ROBOT_JOINTS['ur5'], True)
        for center, euler, half_extents in self.cube_obstacles:
            sim.add_cuboid(half_extents, center, euler)
        for center, radius in self.sphere_obstacles:
            sim.add_sphere(radius, center)
        sim.set_camera(position=[2, -3, -2], look_at=[0, 0, 0])
        sim.animate(plan)

    def which_collides(self, config, obstacle_dict):
        """
        For cuboid obstacles defined using points in real coords
        """
        collides_map = {}
        for name, obstacle in obstacle_dict.items():
            tmp_env = vamp.Environment()
            center, euler, half_extents = _get_cuboid_transform(obstacle)
            tmp_env.add_cuboid(vamp.Cuboid(center, euler, half_extents))
            if self.vamp_module.validate(config, tmp_env):
                collides_map[name] = False
            else:
                collides_map[name] = True
        return collides_map
    
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
    
        initial_z = 3
        step_size = 0.01
        z = initial_z
        while True:
            print(f"trying z: {z}", end="\r")
            tmp_env = vamp.Environment()
            transformed_points = [transform(point, z) for point in calibration_obstacle]
            center = np.mean(transformed_points, axis=0)
            half_extents = np.max(np.abs(transformed_points - center), axis=0)
            euler = [0, 0, 0]
            tmp_env.add_cuboid(vamp.Cuboid(center, euler, half_extents))
            if not self.vamp_module.validate(config, tmp_env):
                return z # collision detected, critical z found
            z -= step_size
