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
    [0.57, 0.225, 0.45],
]
ref_rotated_block = [
    [0.45, 0.225, 0.21],
    [0.69, 0.225, 0.21],
    [0.45, 0.345, 0.21],
    [0.69, 0.345, 0.21],
    [0.45, 0.225, 0.45],
    [0.69, 0.225, 0.45],
    [0.45, 0.345, 0.45],
    [0.69, 0.345, 0.45],
]
ref_standing_block = [
    [0.33, -0.275, 0.21],
    [0.45, -0.275, 0.21],
    [0.33, -0.395, 0.21],
    [0.45, -0.395, 0.21],
    [0.33, -0.275, 0.72],
    [0.45, -0.275, 0.72],
    [0.33, -0.395, 0.72],
    [0.45, -0.395, 0.72],
]
big_standing_block = [
    [0.33, -0.275, 0.21],
    [0.57, -0.275, 0.21],
    [0.33, -0.395, 0.21],
    [0.57, -0.395, 0.21],
    [0.33, -0.275, 0.69],
    [0.57, -0.275, 0.69],
    [0.33, -0.395, 0.69],
    [0.57, -0.395, 0.69],
]
# cube 1 is shifted by +0.12 in x
block1 = np.array(ref_block)
block2 = np.array(ref_block) + np.array([0.12, 0, 0])
block3 = np.array(ref_block) + np.array([0, 0, 0.24])
block4 = np.array(ref_rotated_block)
block5 = np.array(ref_rotated_block) + np.array([0, 0, 0.24])
block6 = np.array(ref_standing_block)  # + np.array([0.12, 0, 0])
block7 = np.array(ref_standing_block) + np.array([0.12, 0, 0])
# wall at y = 31
wall = np.array(
    [
        [-1, 0.22, 0.21],
        [1.2, 0.22, 0.21],
        [-1, 0.35, 0.21],
        [1.2, 0.35, 0.21],
        [-1, 0.22, 2],
        [1.2, 0.22, 2],
        [-1, 0.35, 2],
        [1.2, 0.35, 2],
    ]
)
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

working_space_coords = np.array([
    [0.0, -0.5, 0.21],
    [1.5, -0.5, 0.21],
    [1.5, 0.5, 0.21],
    [0.0, 0.5, 0.21], 
    [0.0, -0.5, 1.5],
    [1.5, -0.5, 1.5],
    [1.5, 0.5, 1.5],
    [0.0, 0.5, 1.5]
])

obstacle_blocks = [block1, block2, block3, block4, block5, wall, floor, block6, block7]
named_blocks = {
    "block1": block1,
    "block2": block2,
    "block3": block3,
    "block4": block4,
    "block5": block5,
    "block6": block6,
    "block7": block7,
    "wall": wall,
    "floor": floor,
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

space_coords = [
    [-0.1, 0.5, 0.2],
    [1.5, 0.5, 0.2],
    [-0.1, 0.4, 0.2],
    [1.5, 0.4, 0.2],
    [-0.1, 0.5, 1.2],
    [1.5, 0.5, 1.2],
    [-0.1, 0.4, 1.2],
    [1.5, 0.4, 1.2],
]

z_offset = 0.6

ref_point_config = [
    0.16554054617881775,
    -2.233336111108297,
    -1.523730754852295,
    -0.9727459710887452,
    1.5914708375930786,
    0.17196546494960785,
]
calibration_config = [
    -0.5705197493182581,
    -2.230887075463766,
    -0.8716913461685181,
    -1.5791627369322718,
    -0.008387390767232716,
    0.001073598861694336,
]
target_pose = [0.68, 0.0, 0.375, -1.209, 1.217, 1.119]
target_pose_1 = [
    0.7138968303965468,
    0.19039962771644575,
    0.30780391940111737,
    -0.41768127619190976,
    2.0086644368765993,
    1.7581360054108013,
]
target_pose_2 = [
    0.5605878845302544,
    -0.4667249338408572,
    0.3052064381230434,
    -1.0260661757836198,
    2.926744903406356,
    0.03804115154885197,
]
target_pose_3 = [
    0.5395072674273982,
    -0.5796117338252653,
    0.18674302499485484,
    1.1418355110009042,
    1.2021400768723933,
    1.1110049425601551,
]

pose_list_1 = [
    [
        0.43382658403689767,
        0.11293014756325452,
        0.11585596995135686,
        2.7385691684254874,
        -1.5153156502400842,
        -0.007055721862769302,
    ],
    [
        0.3179771141404939,
        -0.2645748154513566,
        0.05797334042864255,
        -1.7831768421823502,
        2.5826635867787644,
        -0.009660161065769751,
    ],
    [
        0.8479592435473128,
        -0.24629679204946928,
        0.05019497097617581,
        -2.060300216100177,
        2.3386541392171774,
        -0.07268070517214152,
    ],
    [
        0.8479592435473128,
        -0.24629679204946928,
        0.05019497097617581,
        -2.060300216100177,
        2.3386541392171774,
        -0.07268070517214152,
    ],
    [
        0.8479592435473128,
        -0.24629679204946928,
        0.05019497097617581,
        -2.060300216100177,
        2.3386541392171774,
        -0.07268070517214152,
    ],
    [
        0.8479592435473128,
        -0.24629679204946928,
        0.05019497097617581,
        -2.060300216100177,
        2.3386541392171774,
        -0.07268070517214152,
    ],
    [
        0.8479592435473128,
        -0.24629679204946928,
        0.05019497097617581,
        -2.060300216100177,
        2.3386541392171774,
        -0.07268070517214152,
    ],
]
