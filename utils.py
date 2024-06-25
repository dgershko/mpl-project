import numpy as np

def calculate_center(points):
    return np.mean(points, axis=0)

def calculate_half_extents(points, center):
    half_extents = np.max(np.abs(points - center), axis=0)
    return half_extents

def calculate_euler_angles():
    # Assuming axis-aligned cuboid for simplicity
    return [0, 0, 0]

def calculate_cuboid(points):
    """
    return center, eurler angles, half_extents
    """
    center = calculate_center(points)
    half_extents = calculate_half_extents(points, center)
    euler_angles = calculate_euler_angles()
    return center, euler_angles, half_extents

def transform_to_vamp_coords(point):
    # translate by -0.85 in z
    point = np.array(point) + np.array([0, 0, 0.71])
    # rotate 90 degrees clockwise around z axis
    point = np.array([point[1], -point[0], point[2]])
    return point
