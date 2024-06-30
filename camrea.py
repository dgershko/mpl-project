import pyrealsense2 as rs
import numpy as np
import cv2

from env_consts import working_space_coords, obstacle_blocks

class rsCamera():
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()

        if self.depth_sensor.supports(rs.option.depth_units):
            self.depth_sensor.set_option(rs.option.depth_units, 0.001)

        self.depth_sensor.set_option(rs.option.visual_preset, 3)
        self.depth_scale = self.depth_sensor.get_depth_scale()
        clipping_distance_in_meters = 0.8
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.seen_obstacles = np.array(obstacle_blocks)

    def get_obstacles(self, pose):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        grey_color = 153
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        bg_removed = np.where((depth_image_3d > self.clipping_distance) | \
                (depth_image_3d <= 0), grey_color, color_image)

        bounding_rectangles = detect_obstacle(bg_removed)

        obstacles = []

        for (x, y, w, h) in bounding_rectangles:
            temp = []
            cx_mid, cy_mid = x + w // 2, y + h // 2
            if 0 <= cy_mid <= depth_image.shape[0] and 0 <= cx_mid <= depth_image.shape[1]:
                mid_depth = aligned_depth_frame.get_distance(cx_mid, cy_mid)
                corners_2d = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
                for (cx, cy) in corners_2d:
                    corner_3d_world = get_world_coordinates(color_frame.profile.as_video_stream_profile().intrinsics, pose, cx, cy, mid_depth)
                    corner_3d_world_back = get_world_coordinates(color_frame.profile.as_video_stream_profile().intrinsics, pose, cx, cy, mid_depth + 0.05)
                    temp.append(corner_3d_world)
                    temp.append(corner_3d_world_back)
                obstacles.append(temp.copy())
        obstacles = np.array(obstacles)
        print(f"Initial obstacles: {obstacles}")
        obstacles = filter_obstacles_in_space(obstacles, np.array(working_space_coords))
        print(f"Obstacles after filtering with space: {obstacles}")
        obstacles = filter_seen_obstacles(self.seen_obstacles, obstacles)
        print(f"Obstacles after filtering with other obstacles: {obstacles}")
        if len(obstacles) != 0:
            self.seen_obstacles = np.concatenate((self.seen_obstacles,obstacles))
        return obstacles
    
def detect_obstacle(bg_removed, grey_color=(153, 153, 153)):
    """
    Detecting objects on the image with background removed (all the objects further then the clipping_distance)
    Return list of rectengulars around objects
    """
    mask = cv2.inRange(bg_removed, np.array(grey_color) - 10, np.array(grey_color) + 10)
    mask = cv2.bitwise_not(mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    bounding_rectangles = []
    for contour in contours:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            bounding_rect = (x, y, w, h)
            bounding_rectangles.append(bounding_rect)

    return bounding_rectangles


def get_world_coordinates(intrinsics, camera_pose, x, y, depth):
    """
    Calculating real world coordinates using Eular rotation matrix, depend on position of the camera in space, rotation of the camera,
    placment of the object on the image and depth to the object.
    Returns coordinates [x,y,z]
    """
    X = (x - intrinsics.ppx) / intrinsics.fx * depth
    Y = (y - intrinsics.ppy) / intrinsics.fy * depth
    Z = depth

    camera_position = np.array(camera_pose[:3])

    rx, ry, rz = np.array(camera_pose[3:])
    rotation_matrix = np.array([[np.cos(ry) * np.cos(rz), np.cos(ry) * np.sin(rz), -np.sin(ry)], 
                                [np.sin(rx) * np.sin(ry) * np.cos(rz) - np.cos(rx) * np.sin(rz), np.sin(rx) * np.sin(ry) * np.sin(rz) + np.cos(rx) * np.cos(rz), np.sin(rx) * np.cos(ry)],
                                [np.cos(rx) * np.sin(ry) * np.cos(rz) + np.sin(rx) * np.sin(rz), np.cos(rx) * np.sin(ry) * np.sin(rz) - np.sin(rx) * np.cos(rz), np.cos(rx) * np.cos(ry)]])
    
    point_camera = np.array([X, Y, Z])
    
    point_world = rotation_matrix.dot(point_camera) + camera_position
    return point_world


def is_obstacle_in_space(obstacle, space):
    x_min, y_min, z_min = np.min(space, axis=0)
    x_max, y_max, z_max = np.max(space, axis=0)

    for corner in obstacle:
        if not (x_min <= corner[0] <= x_max and y_min <= corner[1] <= y_max and z_min <= corner[2] <= z_max):
            return False
    return True


def filter_obstacles_in_space(obstacles, space):
    """
    Checks if the obstacles are in workspace.
    Returns list of obstacles in workspace
    """
    inside_obstacles = []

    for obstacle in obstacles:
        if is_obstacle_in_space(obstacle, space):
            inside_obstacles.append(obstacle)

    return np.array(inside_obstacles)

def filter_seen_obstacles(seen_obstacles, obstacles):
    """
    Checks if obstacles that on image are not obstacles that already seen, or initial obstacles.
    Returns list of new obstacles, that never seen before
    """
    outside_obstacles = []
    for obstacle in obstacles:
        intersects = False
        for seen_obstacle in seen_obstacles:
            if is_obstacle_in_space(obstacle, seen_obstacle):
                intersects = True
                break
            if not intersects:
                outside_obstacles.append(obstacle)
    return np.array(outside_obstacles)