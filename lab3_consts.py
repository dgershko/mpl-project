import numpy as np


home = np.deg2rad([0, -90, 0, -90, 0, 0])
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

init_coords = cube_coords.copy()
cube_approaches = [
    np.deg2rad([68.8, -68.3, 84.2, -107.1, -90, -18]),
    np.deg2rad([87.5, -45.5, 47.7, -102, -90.6, 3.3]),
    np.deg2rad([79.7, -46.9, 69.7, -105.1, -92.6, -10.1]),
    np.deg2rad([97.6, -38.3, 52.1, -100.8, -90.1, 8.5]),
    np.deg2rad([104.6, -85.3, 87.7, -90.5, -88.3, 17]),
    np.deg2rad([78.3, -61.7, 120.9, -87.6, -12.9, 27.7]),
]
cubes_actual = [
    np.deg2rad([69, -63, 85, -107.7, -91.3, -18.2]),
    np.deg2rad([86.9, -40.1, 47.7, -102, -90.6, 3.3]),
    np.deg2rad([80.2, -43.4, 68.9, -107.1, -93.9, -9.4]),
    np.deg2rad([97.6, -34.6, 52.3, -100.8, -90.1, 8.5]),
    np.deg2rad([105.1, -86.3, 97.4, -102, -89.8, 19.3]),
    np.deg2rad([78, -56.6, 120.9, -93, -12.7, 29.4]),
]
barakat_cubes = [
    [-0.20, -0.21, 0],
    [-0.20, -0.36, 0],
    [-0.20, -0.51, 0],
    [-0.04, -0.36, 0],
    [-0.04, -0.51, 0],
    [0.01,  -0.40, 0],
]
dgershko_cubes = [
    [-0.205, -0.475, 0],
    [-0.115, -0.475, 0],
    [-0.115, -0.415, 0],
    [-0.205, -0.355, 0],
    [-0.115, -0.355, 0],
    [-0.115, -0.265, 0],
]
roman_cubes = [
    [-0.20, -0.51, 0],
    [-0.20, -0.36, 0],
    [-0.20, -0.25, 0],
    [-0.15, -0.25, 0],
    [-0.10, -0.25, 0],
    [-0.05,  -0.30, 0],
]