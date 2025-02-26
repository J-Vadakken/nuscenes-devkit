from rosbags.typesys import Stores, get_types_from_msg, get_typestore

typestore = get_typestore(Stores.ROS2_HUMBLE)
add_types = {}

RADAR_OBJECT_LIST_MSG = """
std_msgs/Header header
builtin_interfaces/Time processed_at
RadarObject[] radar_objects        
"""
add_types.update(get_types_from_msg(
        RADAR_OBJECT_LIST_MSG, 'autoronto_msgs/msg/RadarObjectList'))

RADAR_OBJECT_MSG = """
float32 id
float32 age
float32 position_x
float32 position_x_std
float32 position_y
float32 position_y_std
float32 position_z
float32 position_z_std
float32 position_covariancexy
float32 position_orientation
float32 position_orientation_std
float32 classification_car
float32 classification_truck
float32 classification_motorcycle
float32 classification_bicycle
float32 classification_pedestrian
float32 classification_animal
float32 classification_hazard
float32 classification_unknown
float32 classification_overdrivable
float32 classification_underdrivable
float32 relvel_x
float32 relvel_x_std
float32 relvel_y
float32 relvel_y_std
float32 relvel_covariancexy
float32 relaccel_x
float32 relaccel_x_std
float32 relaccel_y
float32 relaccel_y_std
float32 mean_length
float32 mean_width    
"""
add_types.update(get_types_from_msg(
        RADAR_OBJECT_MSG, 'autoronto_msgs/msg/RadarObject'))

TRACKING_3D_MSG = """
std_msgs/Header header
builtin_interfaces/Time processed_at
TrackedBox3D[] bbs
"""
add_types.update(get_types_from_msg(
        TRACKING_3D_MSG, 'autoronto_msgs/msg/Tracking3D'))

# TRACKED_BOX_3D_MSG = """
# std_msgs/Header header

# uint64 tracker_id

# # Bounding Box Information
# geometry_msgs/Point center
# float64 yaw
# geometry_msgs/Vector3 velocity
# geometry_msgs/Vector3 size

# # Class Information
# uint32 type
# float64 confidence

# # Useful Contents
# string[] observed_by

# bool is_static
# """
# TRACKED_BOX_3D_MSG = """
# std_msgs/Header header

# uint64 tracker_id

# # Bounding Box Information
# geometry_msgs/Point center
# float64 yaw
# geometry_msgs/Vector3 velocity
# geometry_msgs/Vector3 size

# # Class Information
# uint32 type
# float64 confidence

# # Useful Contents
# string[] observed_by

# bool is_static

# bool seen_by_2d
# bool seen_by_radar
# bool seen_by_car
# bool seen_by_ped
# bool seen_by_deer
# uint64 v_threshold_count
# """
TRACKED_BOX_3D_MSG = """
std_msgs/Header header

uint64 tracker_id

# Bounding Box Information
geometry_msgs/Point center
float64 yaw
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size

# Class Information
uint32 type
float64 confidence

# Useful Contents
string[] observed_by

bool is_static

bool seen_by_2d
bool seen_by_radar
bool seen_by_car
bool seen_by_ped
bool seen_by_deer
uint64 v_threshold_count

bool in_fov

bool in_lane
"""
add_types.update(get_types_from_msg(
        TRACKED_BOX_3D_MSG, 'autoronto_msgs/msg/TrackedBox3D'))

typestore.register(add_types)

# tracked_boxes = typestore.deserialize_cdr(rawdata, typestore.types['autoronto_msgs/msg/Tracking3D'].__msgtype__)
