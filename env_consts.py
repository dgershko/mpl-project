import numpy as np

ref_cube = [
    [0.45, -0.015, 0.21],
    [0.45, 0.225, 0.21],
    [0.57, -0.015, 0.21],
    [0.57, 0.225, 0.21],
    [0.45, -0.015, 0.45],
    [0.45, 0.225, 0.45],
    [0.57, -0.015, 0.45],
    [0.57, 0.225, 0.45]
]
ref_rotated_cube = [
    [0.45, 0.225, 0.21],
    [0.69, 0.225, 0.21],
    [0.45, 0.345, 0.21],
    [0.69, 0.345, 0.21],
    [0.45, 0.225, 0.45],
    [0.69, 0.225, 0.45],
    [0.45, 0.345, 0.45],
    [0.69, 0.345, 0.45]
]
ref_standing_cube = [
    [0.33, -0.275, 0.21],
    [0.45, -0.275, 0.21],
    [0.33, -0.395, 0.21],
    [0.45, -0.395, 0.21],
    [0.33, -0.275, 0.66],
    [0.45, -0.275, 0.66],
    [0.33, -0.395, 0.66],
    [0.45, -0.395, 0.66]
]
# cube 1 is shifted by +0.12 in x
cube1 = np.array(ref_cube)
cube2 = np.array(ref_cube) + np.array([0.12, 0, 0])
cube3 = np.array(ref_cube) + np.array([0, 0, 0.24])
cube4 = np.array(ref_rotated_cube)
cube5 = np.array(ref_rotated_cube) + np.array([0, 0, 0.24])
cube6 = np.array(ref_standing_cube)# + np.array([0.12, 0, 0])
cube7 = np.array(ref_standing_cube) + np.array([0.12, 0, 0])
# wall at y = 31
wall = np.array([
    [-1, 0.22, 0.21],
    [1.2, 0.22, 0.21],
    [-1, 0.35, 0.21],
    [1.2, 0.35, 0.21],
    [-1, 0.22, 2],
    [1.2, 0.22, 2],
    [-1, 0.35, 2],
    [1.2, 0.35, 2]
])
# floor at z = 0.21, 3m sides centered at (0, 0)
floor = np.array([
    [1.5, -1.5, 0.19],
    [1.5, 1.5, 0.19],
    [-1.5, -1.5, 0.19],
    [-1.5, 1.5, 0.19],
    [1.5, -1.5, 0.21],
    [1.5, 1.5, 0.21],
    [-1.5, -1.5, 0.21],
    [-1.5, 1.5, 0.21]
])

cubes = [
    cube1, 
    cube2, 
    cube3, 
    cube4, 
    cube5, 
    wall, 
    floor, 
    cube6
]
named_cubes = {
    "cube1": cube1,
    "cube2": cube2,
    "cube3": cube3,
    "cube4": cube4,
    "cube5": cube5,
    "cube6": cube6,
    "cube7": cube7,
    "wall": wall,
    "floor": floor
}
# cubes.pop(-1)
# cubes.pop(4)

ref_point_config = [0.16554054617881775, -2.233336111108297, -1.523730754852295, -0.9727459710887452, 1.5914708375930786, 0.17196546494960785]

target_pose = [0.68, 0.0, 0.375, -1.209, 1.217, 1.119]