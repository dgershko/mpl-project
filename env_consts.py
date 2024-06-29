import numpy as np

home = np.deg2rad([0, -90, 0, -90, 0, 0])

ref_block = [
    [0.45, -0.015, 0.21],
    [0.45, 0.225, 0.21],
    [0.57, -0.015, 0.21],
    [0.57, 0.225, 0.21],
    [0.45, -0.015, 0.45],
    [0.45, 0.225, 0.45],
    [0.57, -0.015, 0.45],
    [0.57, 0.225, 0.45]
]
ref_rotated_block = [
    [0.45, 0.225, 0.21],
    [0.69, 0.225, 0.21],
    [0.45, 0.345, 0.21],
    [0.69, 0.345, 0.21],
    [0.45, 0.225, 0.45],
    [0.69, 0.225, 0.45],
    [0.45, 0.345, 0.45],
    [0.69, 0.345, 0.45]
]
ref_standing_block = [
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
block1 = np.array(ref_block)
block2 = np.array(ref_block) + np.array([0.12, 0, 0])
block3 = np.array(ref_block) + np.array([0, 0, 0.24])
block4 = np.array(ref_rotated_block)
block5 = np.array(ref_rotated_block) + np.array([0, 0, 0.24])
block6 = np.array(ref_standing_block)# + np.array([0.12, 0, 0])
block7 = np.array(ref_standing_block) + np.array([0.12, 0, 0])
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

obstacle_blocks = [
    block1, 
    block2, 
    block3, 
    block4, 
    block5, 
    wall, 
    floor, 
    block6
]
named_blocks = {
    "block1": block1,
    "block2": block2,
    "block3": block3,
    "block4": block4,
    "block5": block5,
    "block6": block6,
    "block7": block7,
    "wall": wall,
    "floor": floor
}

cube1_coords = [-0.10959248574268822, -0.6417732149769166, 0.1390226933317033]
cube2_coords = [0.08539928976845282, -0.8370930220946053, 0.13813472317717034]
cube3_coords = [-0.008445229140271685, -0.7365370847309188, 0.00955541284784159]
cube4_coords = [0.23647185443765273, -0.769747539513382, 0.03971366463235271]
cube5_coords = [0.26353072323141574, -0.4629969534200313, 0.2651034131371637]
cube6_coords = [0.26940059242703984, -0.4730222745248458, 0.021688493137064376]
cube_coords = [
    cube1_coords,
    cube2_coords,
    cube3_coords,
    cube4_coords,
    cube5_coords,
    cube6_coords,
]

z_offset = 0.71

ref_point_config = [0.16554054617881775, -2.233336111108297, -1.523730754852295, -0.9727459710887452, 1.5914708375930786, 0.17196546494960785]

target_pose = [0.68, 0.0, 0.375, -1.209, 1.217, 1.119]