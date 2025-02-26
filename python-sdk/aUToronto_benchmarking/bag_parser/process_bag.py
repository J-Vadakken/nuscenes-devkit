from rosbags.rosbag2 import Reader as ROS2Reader
from rosbags.highlevel import AnyReader
from rosbags.serde import deserialize_cdr
import numpy as np
import struct
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib
import cv2
import os
import math
import yaml
from message_definitions import *
from create_kitti_annotations import *
from create_datumaro_annotations import *
import argparse


# read config yaml file
cfg_file = 'configs/config.yaml'
cfg = yaml.load(open(cfg_file), Loader=yaml.FullLoader)

pc_fusion_cfg_file = 'configs/pc_fusion_config.yaml'
pc_fusion_cfg = yaml.load(open(pc_fusion_cfg_file), Loader=yaml.FullLoader)
pc_fusion_cfg = pc_fusion_cfg["pc_fusion"]["ros__parameters"]

# set up directories and static transforms
parser = argparse.ArgumentParser()
parser.add_argument('--rosbag_dir', type=str, required=False, help='Path to the rosbag directory')
args = parser.parse_args()

if args.rosbag_dir:
    rosbag_dir = args.rosbag_dir
else:
    rosbag_dir = cfg['rosbag_dir']

basename = rosbag_dir.split('/')[-1]
dir_path = rosbag_dir[:-(len(basename))]

output_folder = f'{dir_path}extraction_{basename}'
gp_output_folder = f'{dir_path}extraction_WITH_GROUND_PLANE_{basename}'
save_paths = [
    os.path.join(output_folder, 'velodyne_points', 'data'),
    os.path.join(gp_output_folder, 'actual_velodyne_points', 'data'),
    os.path.join(output_folder, 'IMAGE_00', 'data'),
    os.path.join(gp_output_folder, 'velodyne_points', 'data')
]

for path in save_paths:
    os.makedirs(path, exist_ok=True)

output_frame_list = 'frame_list.txt'
output_annotations = 'tracklet_labels.xml'
kitti_detections = []
datumaro_detections = []

lidar_topics = [
    '/p_3dod/gprm/pc_fusion'
]
image_topics = [
    '/camera/center_wide/compressed',
    '/camera/left_wide/compressed',
    '/camera/right_wide/compressed'
]

vehicle_imu = {
    "translation": [0, 0.23, -1.70],
    "quaternion": [0, 0, 0, 1],
    "frame_id": "imu",
    "child_frame_id": "vehicle"
}

camera_left_wide = {
    "translation": [
        -0.05619788870920605, 0.23813967972569458, 2.204022409658017
    ],
    "quaternion": [
        0.6629045232900559, -0.26866042027712794, 0.264272224087807,
        -0.6469461825791883
    ],
    "frame_id": "lidar_frame",
    "child_frame_id": "left_wide"
}

camera_right_wide = {
    "translation": [
        -0.2368504114801596, -0.3762137661387273, 1.825429497397728
    ],
    "quaternion": [
        -0.2702637172074669, 0.669621707944198, -0.6260980882184561,
        0.29422011375888596
    ],
    "frame_id": "lidar_frame",
    "child_frame_id": "right_wide"
}

camera_center_wide = {
    "translation": [0.056534265535821235, 0.02678096915813303, 1.5666450649614705],
    "quaternion": [-0.5055780689770171, 0.4917509922187895, -0.49563992421478414, 0.5068657054345556],
    "frame_id": "lidar_frame",
    "child_frame_id": "center_wide"
}

camera_center_lr = {
    "translation": [0.11978556126740925, -0.18421142249742162, 1.5988443377313737],
    "quaternion": [-0.5075730362137084, 0.49226227509409104, -0.4916939470559002, 0.5082170086258195],
    "frame_id": "lidar_frame",
    "child_frame_id": "center_lr"
}

velodyne_transform = {
    "translation": [-0.251433637, 0.456652, 2.488249663],
    "quaternion": [0.016550283, 0.0111677598, -0.04841093, 0.9986279],
    "frame_id": "lidar_frame",
    "child_frame_id": "velodyne"
}

left_wide_intrinsics = np.array([
    [910.6727, 0.0, 805.326, 0.0],
    [0.0, 910.092, 518.7623, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

right_wide_intrinsics = np.array([
    [930.2271, 0.0, 804.2756, 0.0],
    [0.0, 929.6716, 574.0729, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

center_wide_intrinsics = np.array([
    [901.149173391650, 0.0, 815.106695917737, 0.0],
    [0.0, 899.935884781300, 540.692222459260, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

center_lr_intrinsics = np.array([
    [1647.30544143370, 0.0, 820.799056591034, 0.0],
    [0.0, 1648.78815540396, 532.216116631216, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

image_height, image_width = 1100, 1604
save_index = 0

type_to_class = {
    100: 'Pedestrian',
    101: 'Car',
    102: 'Deer',
    103: 'Barrel',
    104: 'RR_Up',
    105: 'RR_Down',
    106: 'Barricade',
    107: 'Unknown',
    108: 'Invalid',
    200: '5Mph',
    201: '10Mph',
    202: '15Mph',
    203: '20Mph',
    204: '25Mph',
    205: '30Mph',
    206: '35Mph',
    207: '40Mph',
    208: '45Mph',
    209: '50Mph',
    210: '55Mph',
    211: '60Mph',
    212: '65Mph',
    213: '70Mph',
    214: '75Mph',
    215: 'Generic',
    216: 'Invalid',
    300: 'Stop',
    301: 'Yield',
    302: 'Railroad',
    303: 'Invalid'
}


def print_topics(ros2_conns, verbose=True):
    print(f'========== {rosbag_dir} Topics ==========')
    ros2_conns = sorted(ros2_conns, key=lambda x: x.topic)
    if verbose:
        for conn in ros2_conns:
            print(f'{conn.topic}, count: {conn.msgcount}')
    print(f'=======================================')
    print(f'{len(ros2_conns)} topics found.')
    print(f'=======================================')

def check_if_topic_exists(ros2_conns, topics, debug=False):
    topics_dict = {}
    topics_msgcount = {}
    
    # Initialize topics_dict with False for all topics
    for topic in topics:
        topics_dict[topic] = False
    
    # Check for each connection if it matches the topics list
    for conn in ros2_conns:
        if conn.topic in topics_dict:
            topics_dict[conn.topic] = True
            topics_msgcount[conn.topic] = conn.msgcount
    
    if debug:
        # Print header with blue text
        print(f'\033[94m========== {rosbag_dir} Extraction Topics ==========\033[0m')
        
        # Print topic existence status
        for topic, exists in topics_dict.items():
            if exists:
                print(f'\033[94m{topic}: {exists}\033[0m')  # Blue for normal
            else:
                print(f'\n\033[91m[WARNING]: {topic} DOES NOT EXIST\033[0m\n')  # Red for warning
        
        print(f'\033[94m=======================================\033[0m')
        
        # Print message count for each topic
        for topic, count in topics_msgcount.items():
            if count == 0:
                print(f'\n\033[91m[WARNING]: {topic} HAS 0 MESSAGES\033[0m\n')  # Red for warning
            else:
                print(f'\033[94m{topic}: {count} messages\033[0m')  # Blue for normal
        
        print(f'\033[94m=======================================\033[0m')

    return topics_dict



def transform_point_cloud(point_cloud, transform_matrix):
    """
    Applies a 4x4 transformation matrix to a point cloud.

    Parameters:
    - point_cloud: numpy array of shape (N, 3) or (N, 4), representing N points.
    - transform_matrix: 4x4 numpy array representing the transformation matrix.

    Returns:
    - transformed_cloud: numpy array of shape (N, 3) or (N, 4), representing the transformed points.
    """
    # Check if the point cloud has 3 or 4 columns
    if point_cloud.shape[1] == 3:
        # Add a column of ones to make the points homogeneous
        ones = np.ones((point_cloud.shape[0], 1))
        homogeneous_points = np.hstack([point_cloud, ones])
    else:
        # Assume the last column is already the homogeneous coordinate
        homogeneous_points = point_cloud

    # Apply the transformation matrix
    transformed_points = homogeneous_points @ transform_matrix
    
    # Return only the x, y, z (and optional intensity) columns
    return transformed_points[:, :point_cloud.shape[1]].astype(np.float32)

def is_point_cloud(topic):
    return 'PointCloud2' in topic


def point_cloud_msg_to_np_array(msg):
    num_points = msg.width * msg.height
    # Create a NumPy array to hold the point cloud data
    points = np.zeros((num_points, 4), dtype=np.float32)
    # Iterate over each point in the point cloud
    for i in range(num_points):
        # Calculate the offset in the data array
        offset = i * msg.point_step
        # Extract the point data (x, y, z) from the message
        # Assume that the point cloud is in the format of (x, y, z, ...)
        # Adjust the offsets (0, 4, 8) if the format is different
        x, y, z, intensity = struct.unpack_from('ffff', msg.data, offset)
        points[i] = [x, y, z, intensity]
    return points

def velodyne_point_cloud_msg_to_np_array(msg):
    points = []  # Start with an empty list to collect points
    
    # Iterate over each point in the point cloud
    for i in range(msg.width * msg.height):
        # Calculate the offset in the data array
        offset = i * msg.point_step
        
        # Extract the point data (x, y, z, intensity) from the message
        x, y, z, intensity = struct.unpack_from('ffff', msg.data, offset)
        
        # Only append points that are not NaN
        if not any(math.isnan(v) for v in [x, y, z, intensity]):
            points.append([x, y, z, intensity])
    
    # Convert the list to a NumPy array with dtype np.float32
    return np.array(points, dtype=np.float32)


def get_3dbox_locations_on_2d_image(lidar_box, extrinsics, intrinsics):
    '''
    lidar_box: [x, y, z, l, w, h, 0, 0, yaw]
    extrinsics: np.array
    intrinsics: np.array

    compute the eight corners of the 3D bounding box in the image frame, if the coordinate on the image is outside the image, set it to the boundary
    '''

    # First, get the corners of the 3D boxes
    x, y, z, l, w, h, roll, pitch, yaw = lidar_box
    half_l = l / 2.0
    half_w = w / 2.0
    half_h = h / 2.0

    # Local corners
    corners_local = np.array([
        [half_l,  half_w,  half_h],
        [half_l,  half_w, -half_h],
        [half_l, -half_w,  half_h],
        [half_l, -half_w, -half_h],
        [-half_l,  half_w,  half_h],
        [-half_l,  half_w, -half_h],
        [-half_l, -half_w,  half_h],
        [-half_l, -half_w, -half_h],
    ])

    # Rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix
    R = Rz @ Ry @ Rx

    # Rotate and translate corners
    corners_global = np.dot(corners_local, R.T)
    corners_global += np.array([x, y, z])

    # Pad a column of ones for homogeneous coordinates
    corners_global = np.hstack(
        (corners_global, np.ones((corners_global.shape[0], 1))))

    # Project the corners to the image coordinate
    corners_image = np.dot(corners_global, extrinsics.T)

    # Project the corners to pixel coordinates
    corners_pixel = np.dot(corners_image, intrinsics.T)
    corners_pixel = corners_pixel[:, :2] / corners_pixel[:, 2, np.newaxis]

    # Determine which points are within the image boundaries
    valid_points = []
    for point in corners_pixel:
        u, v = point
        if 0 <= u < image_width and 0 <= v < image_height:
            valid_points.append(point)

    # Convert the list of valid points to a numpy array
    valid_points = np.array(valid_points)

    return valid_points


def get_lines_from_points(points):
    """
    Given a shape of (N by 2) u, v coordinates, return all the lines.

    Parameters:
    - points: np.array of shape (N, 2), where each row represents (u, v) coordinates of a point.

    Returns:
    - lines: A list of tuples, where each tuple contains two points representing a line.
             Each point is represented as a tuple (u, v).
    """
    num_points = points.shape[0]

    # Ensure there are at least 2 points to form a line
    if num_points < 2:
        return []

    # Create a list to hold the lines
    lines = []

    # Loop through the points to form lines between them
    for i in range(num_points):
        for j in range(i + 1, num_points):
            pt1 = (int(points[i, 0]), int(points[i, 1]))
            pt2 = (int(points[j, 0]), int(points[j, 1]))
            lines.append((pt1, pt2))

    return lines


def plot_bev_point_cloud_and_images(pts, center_image, left_image, right_image, tracked_dict, tf_static_transform_dict):
    global save_index
    print(f'Plotting BEV point cloud and images {save_index}...')
    lidar_pts_x = pts[:, 0]
    lidar_pts_y = pts[:, 1]

    fig = plt.figure(figsize=(30, 20))
    grid = plt.GridSpec(2, 3, height_ratios=[1, 2])

    # Plot the left image
    # Make a copy of the left image to draw on
    left_image_copy = left_image.copy()
    ax1 = fig.add_subplot(grid[0, 0])
    ax1.imshow(cv2.cvtColor(left_image_copy, cv2.COLOR_BGR2RGB))
    lidar_frame_to_left_wide = tf_static_transform_dict['lidar_frame_to_left_wide']
    for tracker_id, box in tracked_dict.items():
        x, y, z = box['xyz'][0], box['xyz'][1], box['xyz'][2]
        z -= 1.7
        if -y < -50 or -y > 50 or x < 0 or x > 75:
            continue
        l, w, h = box['lwh'][0], box['lwh'][1], box['lwh'][2]
        yaw = box['yaw']
        # yaw -= np.pi / 2  # Check if this adjustment is correct
        lidar_box = [x, y, z, l, w, h, 0, 0, yaw]

        # Debug: print out the lidar_box before projection
        # print(f"Tracker ID {tracker_id}: lidar_box = {lidar_box}")

        corners_in_pixel = get_3dbox_locations_on_2d_image(
            lidar_box, extrinsics=lidar_frame_to_left_wide, intrinsics=left_wide_intrinsics)

        # Debug: print out the corners_in_pixel to verify correct projection
        # print(f"Tracker ID {tracker_id}: corners_in_pixel = {corners_in_pixel}")

        if len(corners_in_pixel) == 0:
            continue
        else:
            lines = get_lines_from_points(corners_in_pixel)

        # Draw the lines on the copy of the image
        for line in lines:
            pt1, pt2 = line
            cv2.line(left_image_copy, pt1, pt2, color=(255, 0, 0), thickness=2)

        if len(corners_in_pixel) > 0:
            pt1 = (int(corners_in_pixel[0][0]), int(corners_in_pixel[0][1]))
            cv2.putText(left_image_copy, f'ID: {tracker_id}', pt1,
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)

    ax1.set_title('Left Image')
    ax1.axis('off')

    # Show the updated image
    ax1.imshow(cv2.cvtColor(left_image_copy, cv2.COLOR_BGR2RGB))

    # Plot center image
    # ax2 = fig.add_subplot(grid[0, 1])
    # ax2.imshow(cv2.cvtColor(center_image, cv2.COLOR_BGR2RGB))
    # ax2.set_title('Center Image')
    # ax2.axis('off')

    center_image_copy = center_image.copy()
    ax2 = fig.add_subplot(grid[0, 1])
    ax2.imshow(cv2.cvtColor(center_image_copy, cv2.COLOR_BGR2RGB))
    lidar_frame_to_center_wide = tf_static_transform_dict['lidar_frame_to_center_wide']
    for tracker_id, box in tracked_dict.items():
        x, y, z = box['xyz'][0], box['xyz'][1], box['xyz'][2]
        z -= 1.7
        # x, y = -y, x  # Verify this step if this is correct
        if -y < -50 or -y > 50 or x < 0 or x > 75:
            continue
        l, w, h = box['lwh'][0], box['lwh'][1], box['lwh'][2]
        yaw = box['yaw']
        # yaw -= np.pi / 2  # Check if this adjustment is correct
        lidar_box = [x, y, z, l, w, h, 0, 0, yaw]

        # Debug: print out the lidar_box before projection
        # print(f"Tracker ID {tracker_id}: lidar_box = {lidar_box}")

        corners_in_pixel = get_3dbox_locations_on_2d_image(
            lidar_box, extrinsics=lidar_frame_to_center_wide, intrinsics=center_wide_intrinsics)

        # Debug: print out the corners_in_pixel to verify correct projection
        # print(f"Tracker ID {tracker_id}: corners_in_pixel = {corners_in_pixel}")

        if len(corners_in_pixel) == 0:
            continue
        else:
            lines = get_lines_from_points(corners_in_pixel)

        # Draw the lines on the copy of the image
        for line in lines:
            pt1, pt2 = line
            cv2.line(center_image_copy, pt1, pt2,
                     color=(255, 0, 0), thickness=2)

        if len(corners_in_pixel) > 0:
            pt1 = (int(corners_in_pixel[0][0]), int(corners_in_pixel[0][1]))
            cv2.putText(center_image_copy, f'ID: {tracker_id}', pt1,
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)

    ax2.set_title('Center Image')
    ax2.axis('off')

    # Show the updated image
    ax2.imshow(cv2.cvtColor(center_image_copy, cv2.COLOR_BGR2RGB))

    # Plot right image
    # ax3 = fig.add_subplot(grid[0, 2])
    # ax3.imshow(cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB))
    # ax3.set_title('Right Image')
    # ax3.axis('off')

    right_image_copy = right_image.copy()
    ax3 = fig.add_subplot(grid[0, 2])
    ax3.imshow(cv2.cvtColor(center_image_copy, cv2.COLOR_BGR2RGB))
    lidar_frame_to_right_wide = tf_static_transform_dict['lidar_frame_to_right_wide']
    for tracker_id, box in tracked_dict.items():
        x, y, z = box['xyz'][0], box['xyz'][1], box['xyz'][2]
        z -= 1.7
        # x, y = -y, x  # Verify this step if this is correct
        if -y < -50 or -y > 50 or x < 0 or x > 75:
            continue
        l, w, h = box['lwh'][0], box['lwh'][1], box['lwh'][2]
        yaw = box['yaw']
        # yaw -= np.pi / 2  # Check if this adjustment is correct
        lidar_box = [x, y, z, l, w, h, 0, 0, yaw]

        # Debug: print out the lidar_box before projection
        # print(f"Tracker ID {tracker_id}: lidar_box = {lidar_box}")

        corners_in_pixel = get_3dbox_locations_on_2d_image(
            lidar_box, extrinsics=lidar_frame_to_right_wide, intrinsics=right_wide_intrinsics)

        # Debug: print out the corners_in_pixel to verify correct projection
        # print(f"Tracker ID {tracker_id}: corners_in_pixel = {corners_in_pixel}")

        if len(corners_in_pixel) == 0:
            continue
        else:
            lines = get_lines_from_points(corners_in_pixel)

        # Draw the lines on the copy of the image
        for line in lines:
            pt1, pt2 = line
            cv2.line(right_image_copy, pt1, pt2,
                     color=(255, 0, 0), thickness=2)

        if len(corners_in_pixel) > 0:
            pt1 = (int(corners_in_pixel[0][0]), int(corners_in_pixel[0][1]))
            cv2.putText(right_image_copy, f'ID: {tracker_id}', pt1,
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)

    ax3.set_title('Right Image')
    ax3.axis('off')

    # Show the updated image
    ax3.imshow(cv2.cvtColor(right_image_copy, cv2.COLOR_BGR2RGB))

    # Plot BEV point cloud
    ax4 = fig.add_subplot(grid[1, :])
    ax4.scatter(-lidar_pts_y, lidar_pts_x, s=1)

    ax4.scatter(0, 0, s=100, c='red', marker='x', label='Ego Vehicle')

    # Plot the tracked boxes and velocity arrows
    for tracker_id, box in tracked_dict.items():
        x, y, z = box['xyz'][0], box['xyz'][1], box['xyz'][2]
        x, y = -y, x
        if x < -50 or x > 50 or y < -5 or y > 75:
            continue
        l, w, h = box['lwh'][0], box['lwh'][1], box['lwh'][2]
        vx, vy = box['velocity'][0], box['velocity'][1]
        vx, vy = -vy, vx
        yaw = box['yaw']  # in radians
        yaw -= np.pi / 2  # correct yaw
        obj_type = box['type']

        # Calculate the four corners of the box
        corners = np.array([
            [-l / 2, -w / 2],
            [-l / 2, w / 2],
            [l / 2, w / 2],
            [l / 2, -w / 2]
        ])

        # Rotate the box according to yaw
        rotation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])
        rotated_corners = np.dot(corners, rotation_matrix.T)

        # Translate the box to the correct position
        translated_corners = rotated_corners + np.array([x, y])

        # Plot the box
        ax4.add_patch(plt.Polygon(translated_corners,
                      edgecolor='black', fill=False))

        # Display tracker ID and type near the box
        text_x = x + l / 2
        text_y = y + w / 2
        ax4.text(text_x, text_y,
                 f'ID: {tracker_id}, {type_to_class[obj_type]}\nvx: {vx:.2f}, vy: {vy:.2f}', color='black', fontsize=12)

    ax4.set_xlim(-50, 50)
    ax4.set_ylim(-5, 75)
    ax4.set_aspect('equal', adjustable='box')
    ax4.grid(False)
    ax4.set_title('BEV Point Cloud')

    plt.tight_layout()
    if not Path('bev_viz').exists():
        Path('bev_viz').mkdir()
    plt.savefig(f'bev_viz/bev_point_cloud_and_image_{save_index}.png')
    plt.clf()
    plt.close(fig)

    save_index += 1
    # breakpoint()


def transform_to_lidar_frame(utm_to_lidar, point):
    # Append 1 to the point for homogeneous transformation
    point_h = np.array([point[0], point[1], point[2], 1.0])
    # Transform the point to the LiDAR frame
    point_lidar_h = np.dot(utm_to_lidar, point_h)
    # Return only the x, y, z components
    return point_lidar_h[:3]


def transform_yaw_to_lidar_frame(utm_to_lidar, yaw):
    # Extract the rotation matrix (3x3 upper left part of the utm_to_lidar)
    rotation_matrix = utm_to_lidar[:3, :3]
    # Apply the rotation matrix to the yaw angle
    # Yaw is typically affected by the rotation about the z-axis, so we extract the z-axis rotation
    yaw_lidar = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]) + yaw
    return yaw_lidar


def transform_velocity_to_lidar_frame(utm_to_lidar, velocity):
    # Extract the rotation matrix (3x3 upper left part of the utm_to_lidar)
    rotation_matrix = utm_to_lidar[:3, :3]
    # Apply the rotation to the velocity vector
    velocity_lidar = np.dot(rotation_matrix, velocity)
    return velocity_lidar


def process_tracked_box(tracked_boxes, tf_static_transform_dict, to_lidar_frame=False):
    utm_to_lidar = get_utm_to_lidar(tf_static_transform_dict)
    boxes = tracked_boxes.bbs
    cur_frame_dict = {}
    for b in boxes:
        cur_frame_dict[b.tracker_id] = {}
        cur_frame_dict[b.tracker_id]['type'] = b.type
        cur_frame_dict[b.tracker_id]['confidence'] = b.confidence
        cur_frame_dict[b.tracker_id]['is_static'] = b.is_static
        cur_frame_dict[b.tracker_id]['observed_by'] = b.observed_by

        if to_lidar_frame:
            cur_frame_dict[b.tracker_id]['xyz'] = transform_to_lidar_frame(
                utm_to_lidar, [b.center.x, b.center.y, b.center.z]
            )
            cur_frame_dict[b.tracker_id]['velocity'] = transform_velocity_to_lidar_frame(
                utm_to_lidar, [b.velocity.x, b.velocity.y, b.velocity.z]
            )
            cur_frame_dict[b.tracker_id]['yaw'] = transform_yaw_to_lidar_frame(
                utm_to_lidar, b.yaw)
        else:
            cur_frame_dict[b.tracker_id]['xyz'] = [
                b.center.x, b.center.y, b.center.z]
            cur_frame_dict[b.tracker_id]['velocity'] = [
                b.velocity.x, b.velocity.y, b.velocity.z]
            cur_frame_dict[b.tracker_id]['yaw'] = b.yaw

        cur_frame_dict[b.tracker_id]['lwh'] = [b.size.x, b.size.y, b.size.z]

    return cur_frame_dict


def prune_outliers(cfg, tracked_dict):

    new_tracked_dict = {}

    for key, value in tracked_dict.items():
        x, y, z, l, w, h, yaw = value['xyz'][0], value['xyz'][1], value['xyz'][
            2], value['lwh'][0], value['lwh'][1], value['lwh'][2], value['yaw']

        distance = math.sqrt(x**2 + y**2 + z**2)
        size = l * w * h

        # 1) filter boxes that are outliers based on distance or size
        if distance > cfg["MAX_DIST"] or \
                x > cfg["MAX_X"] or \
                abs(y) > cfg["MAX_Y"] or \
                z > cfg["MAX_Z"] or \
                size > cfg["MAX_VOL"] or \
                size < cfg["MIN_VOL"] or \
                l > cfg["MAX_L"] or l < cfg["MIN_L"] or \
                w > cfg["MAX_W"] or w < cfg["MIN_W"] or \
                h > cfg["MAX_H"] or h < cfg["MIN_H"]:
            continue

        # 2) adjust boxes to account for time lag
        centroid, stdev = get_centroid_and_stdev(
            point_cloud, [x, y, z, cfg["SEARCH_LEN"], cfg["SEARCH_LEN"], cfg["SEARCH_LEN"], 0.0, 0.0, yaw])
        half_main_diagonal = math.sqrt(l**2 + w**2 + h**2) / 2
        if stdev > cfg["MAX_STDEV_SCALING"] * half_main_diagonal:
            pass
        else:
            # correct the center of the box to the cluster centroid
            x, y, _ = centroid
            value['xyz'] = [x, y, z]

        # 3) reject boxes that contain too few points
        if num_pts_in_bbox(point_cloud, [x, y, z, l, w, h, 0.0, 0.0, yaw]) < cfg["MIN_POINTS_IN_BOX"]:
            continue

        # 4) reject boxes whose size is outside the allowed variance for their class
        type_to_mean_var = {
            100: "ped",
            101: "car",
            102: "deer",
            103: "barrel",
            104: "rail_bar",
            105: "rail_bar",
            106: "barricade",
        }
        # TODO: not the cleanest way...
        for i in range(200, 400):
            type_to_mean_var[i] = "sign"

        class_l, class_w, class_h = pc_fusion_cfg.get(type_to_mean_var[value['type']] + "_mean")
        class_l_var, class_w_var, class_h_var = pc_fusion_cfg.get(type_to_mean_var[value['type']] + "_var")

        if abs(l - class_l) > np.sqrt(class_l_var) + cfg["CLASS_PRIOR_TOL"] or abs(w - class_w) > np.sqrt(class_w_var) + cfg["CLASS_PRIOR_TOL"] or abs(h - class_h) > np.sqrt(class_h_var) + cfg["CLASS_PRIOR_TOL"]:
            continue

        new_tracked_dict[key] = value

    return new_tracked_dict


def process_missing_transform(rotation, translation):
    rot_x, rot_y, rot_z, rot_w = rotation[0], rotation[1], rotation[2], rotation[3]
    translation_x, translation_y, translation_z = translation[0], translation[1], translation[2]

    # Convert quaternion to rotation matrix
    # Quaternion to rotation matrix formula
    R = np.array([
        [1 - 2 * (rot_y**2 + rot_z**2), 2 * (rot_x * rot_y -
                                             rot_z * rot_w), 2 * (rot_x * rot_z + rot_y * rot_w)],
        [2 * (rot_x * rot_y + rot_z * rot_w), 1 - 2 *
         (rot_x**2 + rot_z**2), 2 * (rot_y * rot_z - rot_x * rot_w)],
        [2 * (rot_x * rot_z - rot_y * rot_w), 2 *
         (rot_y * rot_z + rot_x * rot_w), 1 - 2 * (rot_x**2 + rot_y**2)]
    ])

    # Create the 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [translation_x, translation_y, translation_z]

    return T, np.linalg.inv(T)


def process_transform(rotation, translation):
    '''
    Input:
        geometry_msgs__msg__Quaternion(x=-0.015612892640592926, y=-0.02344448079192267, z=0.5118242055426339, w=0.8586283110429345, __msgtype__='geometry_msgs/msg/Quaternion')
        geometry_msgs__msg__Vector3(x=277555.63597040984, y=4686606.418078679, z=271.0787026453763, __msgtype__='geometry_msgs/msg/Vector3')
    Output:
        4 by 4 np array
    '''
    rot_x, rot_y, rot_z, rot_w = rotation.x, rotation.y, rotation.z, rotation.w
    translation_x, translation_y, translation_z = translation.x, translation.y, translation.z

    # Convert quaternion to rotation matrix
    # Quaternion to rotation matrix formula
    R = np.array([
        [1 - 2 * (rot_y**2 + rot_z**2), 2 * (rot_x * rot_y -
                                             rot_z * rot_w), 2 * (rot_x * rot_z + rot_y * rot_w)],
        [2 * (rot_x * rot_y + rot_z * rot_w), 1 - 2 *
         (rot_x**2 + rot_z**2), 2 * (rot_y * rot_z - rot_x * rot_w)],
        [2 * (rot_x * rot_z - rot_y * rot_w), 2 *
         (rot_y * rot_z + rot_x * rot_w), 1 - 2 * (rot_x**2 + rot_y**2)]
    ])

    # Create the 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [translation_x, translation_y, translation_z]
    return T, np.linalg.inv(T)


def get_utm_to_lidar(tf_static_transform_dict):
    utm_to_imu = tf_static_transform_dict['utm_to_imu']
    imu_to_vehicle = tf_static_transform_dict['imu_to_vehicle']
    vehicle_to_lidar = tf_static_transform_dict['vehicle_to_lidar_frame']
    utm_to_lidar = np.dot(vehicle_to_lidar, np.dot(imu_to_vehicle, utm_to_imu))
    print(utm_to_lidar)
    return utm_to_lidar


def save_combined_image(center_image, left_image, right_image, new_name):
    # Convert ROS image messages to OpenCV images
    # center_image_buffer = np.frombuffer(center_image.data, np.uint8)
    # left_image_buffer = np.frombuffer(left_image.data, np.uint8)
    # right_image_buffer = np.frombuffer(right_image.data, np.uint8)

    # cv_center_image = cv2.imdecode(center_image_buffer, cv2.IMREAD_COLOR)
    # cv_left_image = cv2.imdecode(left_image_buffer, cv2.IMREAD_COLOR)
    # cv_right_image = cv2.imdecode(right_image_buffer, cv2.IMREAD_COLOR)

    # Combine the images horizontally
    combined_image = np.hstack((left_image, center_image, right_image))

    # Save the combined image
    img_save_path = os.path.join(
        output_folder, 'IMAGE_00', 'data', f'{new_name}.png')
    cv2.imwrite(img_save_path, combined_image)


def save_both_lidar_data(cfg, fused_pc, fused_pc_with_gprm, velodyne_cloud, center_image, left_image, right_image):
    global save_index

    # Ensure file name has 6 digits (e.g. 000013.bin)
    target_num_digits = 6
    num_digits = len(str(save_index))
    diff = target_num_digits - num_digits
    new_name = (str('0') * diff) + str(save_index)

    # Save point cloud data (without ground plane)
    bin_file_path = os.path.join(
        output_folder, 'velodyne_points', 'data', f"{new_name}.bin")
    with open(bin_file_path, 'wb', buffering=4096) as bin_file:
        bin_file.write(fused_pc.tobytes())

    # Save velodyne cloud data
    if velodyne_cloud is not None:
        bin_file_path_velodyne = os.path.join(
            gp_output_folder, 'actual_velodyne_points', 'data', f"{new_name}.bin")
        with open(bin_file_path_velodyne, 'wb', buffering=4096) as bin_file:
            bin_file.write(velodyne_cloud.tobytes())

    # Save point cloud data (with ground plane)
    bin_file_path_gp = os.path.join(
        gp_output_folder, 'velodyne_points', 'data', f"{new_name}.bin")
    with open(bin_file_path_gp, 'wb', buffering=4096) as bin_file:
        bin_file.write(fused_pc_with_gprm.tobytes())
        
    # Save combined image
    if center_image is not None and left_image is not None and right_image is not None:
        save_combined_image(center_image, left_image, right_image, new_name)

    if not cfg['plot_bev']:
        save_index += 1


def num_pts_in_bbox(point_cloud, lidar_box):
    '''
    Gets the number of points from point cloud contained in the bounding box

    Input:
        point_cloud: np.array of shape (N, 4)
        lidar_box: [x, y, z, l, w, h, 0, 0, yaw]
    Output:
        int
    '''

    # Extract bounding box parameters
    box_x, box_y, box_z, l, w, h, _, _, yaw = lidar_box

    # Compute rotation matrix for the yaw (rotation about the z-axis)
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,        0,       1]
    ])

    # Translate point cloud to bounding box coordinate frame
    translated_points = point_cloud[:, :3] - np.array([box_x, box_y, box_z])

    # # plot the translated points (bev)
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.scatter(translated_points[:, 0], translated_points[:, 1], s=1)
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # plt.show()
    # # save the figure
    # fig.savefig('translated_points.png')

    # Rotate the points into the bounding box's frame
    rotated_points = translated_points @ rotation_matrix.T

    # # plot the rotated points with the box
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.scatter(rotated_points[:, 0], rotated_points[:, 1], s=1)
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # # plot the box
    # box_corners_2d = np.array([
    #     [l/2, w/2],
    #     [l/2, -w/2],
    #     [-l/2, -w/2],
    #     [-l/2, w/2],
    # ])
    # ax.add_patch(plt.Polygon(box_corners_2d,
    #                   edgecolor='black', fill=False))
    # plt.show()
    # # save the figure
    # fig.savefig('rotated_points.png')

    # Check if the points are within the bounding box dimensions
    in_box_x = np.abs(rotated_points[:, 0]) <= l / 2
    in_box_y = np.abs(rotated_points[:, 1]) <= w / 2
    in_box_z = np.abs(rotated_points[:, 2]) <= h / 2

    # Combine the conditions for x, y, and z axes
    in_box = in_box_x & in_box_y  # not incoporating z works better since z floats

    # Count the number of points inside the bounding box
    return np.sum(in_box)


def get_centroid_and_stdev(point_cloud, search_box):
    '''
    Compute the centroid and stdev of the points within the search box

    Input:
        point_cloud: np.array of shape (N, 4)
        search_box: [x, y, z, l, w, h, 0, 0, yaw]
    Output:
        centroid: np.array of shape (3,)
        stdev: float
    '''

    # Extract bounding box parameters
    box_x, box_y, box_z, l, w, h, _, _, yaw = search_box

    # Compute rotation matrix for the yaw (rotation about the z-axis)
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,        0,       1]
    ])

    # Translate point cloud to bounding box coordinate frame
    translated_points = point_cloud[:, :3] - np.array([box_x, box_y, box_z])

    # Rotate the points into the bounding box's frame
    rotated_points = translated_points @ rotation_matrix.T

    # Check if the points are within the bounding box dimensions
    in_box_x = np.abs(rotated_points[:, 0]) <= l / 2
    in_box_y = np.abs(rotated_points[:, 1]) <= w / 2
    in_box_z = np.abs(rotated_points[:, 2]) <= h / 2

    # Combine the conditions for x, y, and z axes
    # in_box = in_box_x & in_box_y & in_box_z
    in_box = in_box_x & in_box_y  # not incoporating z works better since z floats

    # Get the points inside the bounding box
    points_in_box = rotated_points[in_box]

    if len(points_in_box) == 0:
        return np.array([box_x, box_y, box_z]), 0.0
    
    # Compute the centroid of the points
    centroid = np.mean(points_in_box, axis=0)

    # Compute the variance of the points
    variance = np.var(points_in_box, axis=0)
    variance = np.sqrt(variance[0]**2 + variance[1]**2 + variance[2]**2)
    stdev = np.sqrt(variance)

    # debugging only: plot the points in the bounding box in 3d
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(points_in_box[:, 0], points_in_box[:, 1], points_in_box[:, 2])
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # plt.show()
    # # save the figure
    # fig.savefig('points_in_box.png')

    # transform centroid back to the original frame
    centroid = np.dot(centroid, rotation_matrix) + np.array([box_x, box_y, box_z])

    return centroid, stdev


def plot_cloud_bev(velodyne_cloud, point_cloud):
    """
    Plots the bird's eye view of two point clouds with Velodyne points in blue and another point cloud in red.

    Parameters:
    - velodyne_cloud: numpy array of shape (N, 4) representing Velodyne point cloud.
    - point_cloud: numpy array of shape (M, 4) representing another point cloud.
    """
    plt.figure(figsize=(30, 30))
    plt.scatter(velodyne_cloud[:, 0], velodyne_cloud[:, 1], s=1, color='blue', label='Velodyne Cloud')
    plt.scatter(point_cloud[:, 0], point_cloud[:, 1], s=1, color='black', label='Point Cloud')

    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title("Bird's Eye View of Point Clouds")
    plt.legend()
    plt.axis('equal')
    plt.savefig('velodyne_vs_cepton_bev.png')


if __name__ == '__main__':
    # First get all the tf_static transforms
    tf_static_transforms = []
    with AnyReader([Path(rosbag_dir)]) as ros2_reader:
        connections = [
            x for x in ros2_reader.connections if x.topic == '/tf_static']
        print_topics(connections, verbose=False)
        ros2_messages = ros2_reader.messages(connections=connections)

        for (connection, timestamp, rawdata) in ros2_messages:
            tf_static = ros2_reader.deserialize(rawdata, connection.msgtype)
            for transform in tf_static.transforms:
                tf_static_transforms.append(transform)

    tf_static_transform_dict = {}
    for transform in tf_static_transforms:
        print(
            f'Got transformation: {transform.child_frame_id} to {transform.header.frame_id}')
        transformation_name = f'{transform.child_frame_id}_to_{transform.header.frame_id}'
        inverse_transformation_name = f'{transform.header.frame_id}_to_{transform.child_frame_id}'
        transform, inv_transform = process_transform(
            transform.transform.rotation, transform.transform.translation)
        tf_static_transform_dict[transformation_name] = transform
        tf_static_transform_dict[inverse_transformation_name] = inv_transform

    if "vehicle_to_imu" not in tf_static_transform_dict:
        tf_static_transform_dict['vehicle_to_imu'] = process_missing_transform(
            vehicle_imu['quaternion'], vehicle_imu['translation'])[0]
        tf_static_transform_dict['imu_to_vehicle'] = process_missing_transform(
            vehicle_imu['quaternion'], vehicle_imu['translation'])[1]

    if 'lidar_frame_to_left_wide' not in tf_static_transform_dict:
        tf_static_transform_dict['left_wide_to_lidar_frame'] = process_missing_transform(
            camera_left_wide['quaternion'], camera_left_wide['translation'])[0]
        tf_static_transform_dict['lidar_frame_to_left_wide'] = process_missing_transform(
            camera_left_wide['quaternion'], camera_left_wide['translation'])[1]

    if 'lidar_frame_to_right_wide' not in tf_static_transform_dict:
        tf_static_transform_dict['right_wide_to_lidar_frame'] = process_missing_transform(
            camera_right_wide['quaternion'], camera_right_wide['translation'])[0]
        tf_static_transform_dict['lidar_frame_to_right_wide'] = process_missing_transform(
            camera_right_wide['quaternion'], camera_right_wide['translation'])[1]

    if 'lidar_frame_to_center_wide' not in tf_static_transform_dict:
        tf_static_transform_dict['center_wide_to_lidar_frame'] = process_missing_transform(
            camera_center_wide['quaternion'], camera_center_wide['translation'])[0]
        tf_static_transform_dict['lidar_frame_to_center_wide'] = process_missing_transform(
            camera_center_wide['quaternion'], camera_center_wide['translation'])[1]
    
    if 'velodyne_to_lidar_frame' not in tf_static_transform_dict:
        tf_static_transform_dict['lidar_frame_to_velodyne'] = process_missing_transform(
            velodyne_transform['quaternion'], velodyne_transform['translation'])[0]
        tf_static_transform_dict['velodyne_to_lidar_frame'] = process_missing_transform(
            velodyne_transform['quaternion'], velodyne_transform['translation'])[1]
        
    # Then get the corresponding topics
    prev_tracks = {}
    with AnyReader([Path(rosbag_dir)]) as ros2_reader:
        connections = [x for x in ros2_reader.connections]
        print_topics(connections, verbose=True)
        if cfg['generate_autolabels'] and cfg['tracker_topic'] not in connections:
            # print(f'\n************ Extraction FAILED ************')
            # print('REASON: No tracker topic in this bag, you can run record tracker running on this bag locally and run the extraction script on the new bag.')
            # print(f'*******************************************')
            # print('Exiting extraction scipt.')
            # exit()
            print('No tracker topic in this bag, you can run record tracker running on this bag locally and run the extraction script on the new bag. For now, we will not generate autolabels.')
            cfg['generate_autolabels'] = False
        
        extraction_topics = [
            cfg['tracker_topic'],
            cfg['left_camera_topic'],
            cfg['right_camera_topic'],
            cfg['center_camera_topic'],
            cfg['left_raw_lidar_topic'],
            cfg['right_raw_lidar_topic'],
            cfg['center_top_raw_lidar_topic'],
            cfg['center_bottom_raw_lidar_topic'],
            cfg['fused_lidar_topic'],
            cfg['imu_topic'],
            cfg['velodyne_topic'],
            cfg['left_gprm_topic'],
            cfg['right_gprm_topic'],
            cfg['center_top_gprm_topic'],
            cfg['center_bottom_gprm_topic'],
        ]
        
        
        extraction_topic_status = check_if_topic_exists(connections, extraction_topics, debug=True) 

        if extraction_topic_status[cfg['fused_lidar_topic']]:
            extraction_topic_status[cfg['left_gprm_topic']] = False
            extraction_topic_status[cfg['right_gprm_topic']] = False
            extraction_topic_status[cfg['center_top_gprm_topic']] = False
            extraction_topic_status[cfg['center_bottom_gprm_topic']] = False
        
        ros2_messages = ros2_reader.messages(connections=connections)
        point_cloud = None
        center_image = None
        left_image = None
        right_image = None
        tracked_boxes = None
        imu_to_utm = None
        velodyne_cloud = None
        point_cloud_left = None
        point_cloud_right = None
        point_cloud_center_top = None
        point_cloud_center_bottom = None
        point_cloud_left_gprm = None
        point_cloud_right_gprm = None
        point_cloud_center_top_gprm = None
        point_cloud_center_bottom_gprm = None
        # breakpoint()
        frame_idx = 0
        for (connection, timestamp, rawdata) in ros2_messages:
            if connection.topic in lidar_topics and is_point_cloud(connection.msgtype):
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                point_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                point_cloud = point_cloud_msg_to_np_array(point_cloud_message)
            elif connection.topic == cfg['left_raw_lidar_topic']:
                left_point_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                point_cloud_left = point_cloud_msg_to_np_array(
                    left_point_cloud_message)
            elif connection.topic == cfg['right_raw_lidar_topic']:
                right_point_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                point_cloud_right = point_cloud_msg_to_np_array(
                    right_point_cloud_message)
            elif connection.topic == cfg['center_top_raw_lidar_topic']:
                center_top_point_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                point_cloud_center_top = point_cloud_msg_to_np_array(
                    center_top_point_cloud_message)
            elif connection.topic == cfg['center_bottom_raw_lidar_topic']:
                center_bottom_point_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                point_cloud_center_bottom = point_cloud_msg_to_np_array(
                    center_bottom_point_cloud_message)
            elif connection.topic == cfg['center_camera_topic']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                image_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                image_buffer = np.frombuffer(image_message.data, np.uint8)
                center_image = cv2.imdecode(image_buffer, cv2.IMREAD_COLOR)
            elif connection.topic == cfg['left_camera_topic']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                image_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                image_buffer = np.frombuffer(image_message.data, np.uint8)
                left_image = cv2.imdecode(image_buffer, cv2.IMREAD_COLOR)
            elif connection.topic == cfg['right_camera_topic']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                image_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                image_buffer = np.frombuffer(image_message.data, np.uint8)
                right_image = cv2.imdecode(image_buffer, cv2.IMREAD_COLOR)

            elif connection.topic == cfg['tracker_topic'] and cfg['generate_autolabels']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                tracked_boxes = typestore.deserialize_cdr(
                    rawdata, typestore.types['autoronto_msgs/msg/Tracking3D'].__msgtype__)
            elif connection.topic == cfg['imu_topic']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                tf = ros2_reader.deserialize(rawdata, connection.msgtype)
                assert len(tf.transforms) == 1, 'Expected only one transform'
                imu_to_utm = tf.transforms[0]
                if imu_to_utm.child_frame_id == 'imu' and imu_to_utm.header.frame_id == 'utm':
                    print(
                        f'Got transformation: {imu_to_utm.child_frame_id} to {imu_to_utm.header.frame_id}')
                    imu_to_utm_rotation = imu_to_utm.transform.rotation
                    imu_to_utm_translation = imu_to_utm.transform.translation
                    imu_to_utm_transform, utm_to_imu_transform = process_transform(
                        imu_to_utm_rotation, imu_to_utm_translation)
                    tf_static_transform_dict['imu_to_utm'] = imu_to_utm_transform
                    tf_static_transform_dict['utm_to_imu'] = utm_to_imu_transform
                else:
                    imu_to_utm = None
            elif connection.topic == cfg['velodyne_topic']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                velodyne_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                velodyne_cloud = velodyne_point_cloud_msg_to_np_array(
                    velodyne_cloud_message)
                # Then transform the velodyne cloud to the lidar frame
                velodyne_to_lidar = tf_static_transform_dict['velodyne_to_lidar_frame']
                lidar_to_velodyne = tf_static_transform_dict['lidar_frame_to_velodyne']
                velodyne_cloud = transform_point_cloud(velodyne_cloud, lidar_to_velodyne)
                
            elif connection.topic == cfg['left_gprm_topic']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                point_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                point_cloud_left_gprm = point_cloud_msg_to_np_array(point_cloud_message)
                
            elif connection.topic == cfg['right_gprm_topic']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                point_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                point_cloud_right_gprm = point_cloud_msg_to_np_array(point_cloud_message)

            elif connection.topic == cfg['center_top_gprm_topic']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                point_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                point_cloud_center_top_gprm = point_cloud_msg_to_np_array(point_cloud_message)            

            elif connection.topic == cfg['center_bottom_gprm_topic']:
                print(
                    f'Got topic: {connection.topic}, message type: {connection.msgtype}')
                point_cloud_message = ros2_reader.deserialize(
                    rawdata, connection.msgtype)
                point_cloud_center_bottom_gprm = point_cloud_msg_to_np_array(point_cloud_message)          
                
            # try:
                # condition = point_cloud is not None and center_image is not None and left_image is not None and right_image is not None and imu_to_utm is not None\
                #         and point_cloud_left is not None and point_cloud_right is not None and point_cloud_center_top is not None and point_cloud_center_bottom is not None
            condition = (
                (point_cloud is not None or not extraction_topic_status['/p_3dod/gprm/pc_fusion']) and
                (center_image is not None or not extraction_topic_status[cfg['center_camera_topic']]) and
                (left_image is not None or not extraction_topic_status[cfg['left_camera_topic']]) and
                (right_image is not None or not extraction_topic_status[cfg['right_camera_topic']]) and
                (tracked_boxes is not None or not extraction_topic_status[cfg['tracker_topic']] or not cfg['generate_autolabels']) and
                (point_cloud_left is not None or not extraction_topic_status[cfg['left_raw_lidar_topic']]) and
                (point_cloud_right is not None or not extraction_topic_status[cfg['right_raw_lidar_topic']]) and
                (point_cloud_center_top is not None or not extraction_topic_status[cfg['center_top_raw_lidar_topic']]) and
                (point_cloud_center_bottom is not None or not extraction_topic_status[cfg['center_bottom_raw_lidar_topic']]) and
                (point_cloud_left_gprm is not None or not extraction_topic_status[cfg['left_gprm_topic']]) and 
                (point_cloud_right_gprm is not None or not extraction_topic_status[cfg['right_gprm_topic']]) and 
                (point_cloud_center_top_gprm is not None or not extraction_topic_status[cfg['center_top_gprm_topic']]) and 
                (point_cloud_center_bottom_gprm is not None or not extraction_topic_status[cfg['center_bottom_gprm_topic']])
                # (imu_to_utm is not None or not extraction_topic_status[cfg['imu_topic']])
            )
            # except NameError:
            #     continue
            if cfg['generate_autolabels'] and condition and tracked_boxes is not None:
                tracked_dict = process_tracked_box(
                    tracked_boxes, tf_static_transform_dict, to_lidar_frame=True)

                # prune outlier boxes
                tracked_dict = prune_outliers(cfg, tracked_dict)

                # kitti_detections.extend(convert_to_kitti_format(tracked_dict, frame_idx))
                datumaro_detections.append(generate_annotation(
                    frame_idx, tracked_dict, prev_tracks))
                prev_tracks = tracked_dict

                if cfg['plot_bev']:
                    plot_bev_point_cloud_and_images(
                        point_cloud, center_image, left_image, right_image, tracked_dict, tf_static_transform_dict)

            if condition:
                # plot_cloud_bev(velodyne_cloud, point_cloud)
                # fused_lidar_with_gp = np.concatenate(
                #     (point_cloud_left, point_cloud_right, point_cloud_center_top, point_cloud_center_bottom), axis=0)
                point_clouds = [point_cloud_left, point_cloud_right, point_cloud_center_top, point_cloud_center_bottom]

                # Use filter to only include non-empty point clouds
                valid_point_clouds = list(filter(lambda pc: pc is not None, point_clouds))

                # If there are valid point clouds, concatenate them, otherwise handle the empty case
                if valid_point_clouds:
                    fused_lidar_with_gp = np.concatenate(valid_point_clouds, axis=0)
                else:
                    fused_lidar_with_gp = np.array([])  # Handle empty case (e.g., return an empty array)
                    print("Warning: All point clouds are empty.")

                print(f"Fused lidar with GP has {fused_lidar_with_gp.shape[0]} points.")
                
                if point_cloud is None:
                    print('\n\033[91m[WARNING]: No directly fused point cloud, manually fusing separate gprm topics.\033[0m\n')
                    
                    gprm_point_clouds = [point_cloud_left_gprm, point_cloud_right_gprm, point_cloud_center_top_gprm, point_cloud_center_bottom_gprm]

                    # Use filter to only include non-empty point clouds
                    gprm_valid_point_clouds = list(filter(lambda pc: pc is not None, gprm_point_clouds))

                    # If there are valid point clouds, concatenate them, otherwise handle the empty case
                    if gprm_valid_point_clouds:
                        point_cloud = np.concatenate(gprm_valid_point_clouds, axis=0)
                    else:
                        point_cloud = np.array([])  # Handle empty case (e.g., return an empty array)
                        
                    # Print the message with the number of points in the fused lidar (normal text in blue)
                    print(f"\033[91mFused lidar with GPRM has {point_cloud.shape[0]} points.\033[0m")

                    
                save_both_lidar_data(
                    cfg, point_cloud, fused_lidar_with_gp, velodyne_cloud, center_image, left_image, right_image)
                point_cloud = None
                center_image = None
                left_image = None
                right_image = None
                tracked_boxes = None
                imu_to_utm = None
                tracked_dict = None
                imu_to_utm = None
                velodyne_cloud = None
                point_cloud_left = None
                point_cloud_right = None
                point_cloud_center_top = None
                point_cloud_center_bottom = None
                point_cloud_left_gprm = None
                point_cloud_right_gprm = None
                point_cloud_center_top_gprm = None
                point_cloud_center_bottom_gprm = None
                frame_idx += 1
                print(f'\033[92mProcessed frame {frame_idx}\033[0m')

    # create_tracklet_xml(kitti_detections, output_annotations)
    # generate_frame_list(output_frame_list, frame_idx)
    datumaro_annotations = generate_datumaro_annotations(datumaro_detections)
    with open("annotations/default.json", "w") as f:
        json.dump(datumaro_annotations, f, indent=4)
    print(f'Finished processing {frame_idx+1} frames.')
