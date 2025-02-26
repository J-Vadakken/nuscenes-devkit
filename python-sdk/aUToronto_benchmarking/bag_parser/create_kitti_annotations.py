import xml.etree.ElementTree as ET
import numpy as np

# Define the constants
STATE_UNSET = 0
STATE_INTERP = 1
STATE_LABELED = 2

OCC_UNSET = 255  # -1 as uint8
OCC_VISIBLE = 0
OCC_PARTLY = 1
OCC_FULLY = 2

TRUNC_UNSET = 255  # -1 as uint8, but in xml files the value '99' is used!
TRUNC_IN_IMAGE = 0
TRUNC_TRUNCATED = 1
TRUNC_OUT_IMAGE = 2
TRUNC_BEHIND_IMAGE = 3

TYPE_TO_CVAT_CLASS = {
    100: 'pedestrian',
    101: 'car',
    102: 'deer',
    103: 'construction_barrel',
    104: 'railroad_bar_up',
    105: 'railroad_bar_down',
    106: 'type_3_barricade',
    200: 'traffic_sign',
    201: 'traffic_sign',
    202: 'traffic_sign',
    203: 'traffic_sign',
    204: 'traffic_sign',
    205: 'traffic_sign',
    206: 'traffic_sign',
    207: 'traffic_sign',
    208: 'traffic_sign',
    209: 'traffic_sign',
    210: 'traffic_sign',
    211: 'traffic_sign',
    212: 'traffic_sign',
    213: 'traffic_sign',
    214: 'traffic_sign',
    215: 'traffic_sign',
    300: 'stop_sign',
    301: 'yield_sign',
    302: 'railroad_sign',
}

def convert_to_kitti_format(detections, frame_idx):
    kitti_list = []
    # Example usage
    # detections = [
    #     {
    #         'objectType': 'fire_hydrant',
    #         'size': [0.68, 0.6, 1.08],  # height, width, length
    #         'firstFrame': 2,
    #         'trans': [[12.78, 11.04, 1.18]],  # x, y, z
    #         'rots': [[0.0, 0.0, 0.0]],  # rx, ry, rz
    #         'states': [STATE_LABELED],
    #         'occs': [[OCC_VISIBLE, OCC_VISIBLE]],
    #         'truncs': [TRUNC_IN_IMAGE],
    #         'amtOccs': [[-1, -1]],
    #         'amtBorders': [[-1, -1, -1]],
    #     },
    #     {
    #         'objectType': 'fire_hydrant',
    #         'size': [0.68, 0.6, 1.08],
    #         'firstFrame': 1,
    #         'trans': [[12.78, 11.04, 1.18]],
    #         'rots': [[0.0, 0.0, 0.0]],
    #         'states': [STATE_LABELED],
    #         'occs': [[OCC_VISIBLE, OCC_PARTLY]],
    #         'truncs': [TRUNC_IN_IMAGE],
    #         'amtOccs': [[-1, -1]],
    #         'amtBorders': [[-1, -1, -1]],
    #     },
    #     {
    #         'objectType': 'fire_hydrant',
    #         'size': [0.68, 0.6, 1.08],
    #         'firstFrame': 0,
    #         'trans': [[12.84, 10.96, 1.18]],
    #         'rots': [[0.0, 0.0, 0.0]],
            # 'states': [STATE_LABELED],
            # 'occs': [[OCC_VISIBLE, OCC_PARTLY]],
            # 'truncs': [TRUNC_IN_IMAGE],
            # 'amtOccs': [[-1, -1]],
            # 'amtBorders': [[-1, -1, -1]],
    #     }
    # ]
    for track_id, box in detections.items():
        x, y, z = box['xyz'][0], box['xyz'][1], box['xyz'][2] - 1.7
        # x, y = -y, x
        l, w, h = box['lwh'][0], box['lwh'][1], box['lwh'][2]
        yaw = box['yaw']
        yaw -= np.pi / 2 # correct yaw
        if box['type'] in TYPE_TO_CVAT_CLASS:
            object_type = TYPE_TO_CVAT_CLASS[box['type']]
        else:
            continue
        detection = {
            'objectType': object_type,
            'size': [l, w, h],
            'firstFrame': frame_idx,
            'trans': [[x, y, z]],
            'rots': [[0, 0, yaw]],
            'states': [STATE_LABELED],
            'occs': [[OCC_VISIBLE, OCC_PARTLY]],
            'truncs': [TRUNC_IN_IMAGE],
            'amtOccs': [[-1, -1]],
            'amtBorders': [[-1, -1, -1]],
        }
        kitti_list.append(detection)
    return kitti_list

def create_tracklet_xml(detections, output_file):
    """
    Create a KITTI format XML from a list of detections.
    
    :param detections: List of dictionaries containing detection information.
                       Each dictionary should have the following keys:
                       - objectType
                       - size (height, width, length)
                       - firstFrame
                       - trans (n x 3 list of (x, y, z))
                       - rots (n x 3 list of (rx, ry, rz))
                       - states (n list)
                       - occs (n x 2 list)
                       - truncs (n list)
                       - amtOccs (n x 2 list or None)
                       - amtBorders (n x 3 list or None)
    :param output_file: The output XML file path.
    """
    # Create the root element
    root = ET.Element('boost_serialization', version="9", signature="serialization::archive")
    
    # Create the tracklets element
    tracklets = ET.SubElement(root, 'tracklets', version="0", tracking_level="0", class_id="0")
    
    # Add the count element
    ET.SubElement(tracklets, 'count').text = str(len(detections))
    
    # Add the item_version element
    ET.SubElement(tracklets, 'item_version').text = "1"
    
    # Iterate over all detections and create tracklet items
    for i, detection in enumerate(detections):
        item = ET.SubElement(tracklets, 'item')
        
        # Set the object type, size, and first frame
        ET.SubElement(item, 'objectType').text = detection['objectType']
        ET.SubElement(item, 'h').text = str(detection['size'][0])
        ET.SubElement(item, 'w').text = str(detection['size'][1])
        ET.SubElement(item, 'l').text = str(detection['size'][2])
        ET.SubElement(item, 'first_frame').text = str(detection['firstFrame'])
        
        # Create poses element
        poses = ET.SubElement(item, 'poses')
        ET.SubElement(poses, 'count').text = str(len(detection['trans']))
        ET.SubElement(poses, 'item_version').text = "0"
        
        for j in range(len(detection['trans'])):
            pose_item = ET.SubElement(poses, 'item')
            ET.SubElement(pose_item, 'tx').text = str(detection['trans'][j][0])
            ET.SubElement(pose_item, 'ty').text = str(detection['trans'][j][1])
            ET.SubElement(pose_item, 'tz').text = str(detection['trans'][j][2])
            ET.SubElement(pose_item, 'rx').text = str(detection['rots'][j][0])
            ET.SubElement(pose_item, 'ry').text = str(detection['rots'][j][1])
            ET.SubElement(pose_item, 'rz').text = str(detection['rots'][j][2])
            ET.SubElement(pose_item, 'state').text = str(detection['states'][j])
            ET.SubElement(pose_item, 'occlusion').text = str(detection['occs'][j][0])
            ET.SubElement(pose_item, 'occlusion_kf').text = str(detection['occs'][j][1])
            ET.SubElement(pose_item, 'truncation').text = str(detection['truncs'][j])
            ET.SubElement(pose_item, 'amt_occlusion').text = str(detection['amtOccs'][j][0] if detection['amtOccs'] is not None else "-1")
            ET.SubElement(pose_item, 'amt_border_l').text = str(detection['amtBorders'][j][0] if detection['amtBorders'] is not None else "-1")
            ET.SubElement(pose_item, 'amt_border_r').text = str(detection['amtBorders'][j][1] if detection['amtBorders'] is not None else "-1")
            ET.SubElement(pose_item, 'amt_occlusion_kf').text = str(detection['amtOccs'][j][1] if detection['amtOccs'] is not None else "-1")
            ET.SubElement(pose_item, 'amt_border_kf').text = str(detection['amtBorders'][j][2] if detection['amtBorders'] is not None else "-1")
        
        # Add the finished element
        ET.SubElement(item, 'finished').text = '1'
    
    # Write to output file with the correct XML declaration and DOCTYPE
    tree = ET.ElementTree(root)
    
    # Manually write the XML file with the DOCTYPE
    with open(output_file, 'wb') as f:
        f.write(b'<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n')
        f.write(b'<!DOCTYPE boost_serialization>\n')
        tree.write(f, encoding='utf-8')

# Example usage
detections = [
    {
        'objectType': 'fire_hydrant',
        'size': [0.68, 0.6, 1.08],  # height, width, length
        'firstFrame': 2,
        'trans': [[12.78, 11.04, 1.18]],  # x, y, z
        'rots': [[0.0, 0.0, 0.0]],  # rx, ry, rz
        'states': [STATE_LABELED],
        'occs': [[OCC_VISIBLE, OCC_VISIBLE]],
        'truncs': [TRUNC_IN_IMAGE],
        'amtOccs': [[-1, -1]],
        'amtBorders': [[-1, -1, -1]],
    },
    {
        'objectType': 'fire_hydrant',
        'size': [0.68, 0.6, 1.08],
        'firstFrame': 1,
        'trans': [[12.78, 11.04, 1.18]],
        'rots': [[0.0, 0.0, 0.0]],
        'states': [STATE_LABELED],
        'occs': [[OCC_VISIBLE, OCC_PARTLY]],
        'truncs': [TRUNC_IN_IMAGE],
        'amtOccs': [[-1, -1]],
        'amtBorders': [[-1, -1, -1]],
    },
    {
        'objectType': 'fire_hydrant',
        'size': [0.68, 0.6, 1.08],
        'firstFrame': 0,
        'trans': [[12.84, 10.96, 1.18]],
        'rots': [[0.0, 0.0, 0.0]],
        'states': [STATE_LABELED],
        'occs': [[OCC_VISIBLE, OCC_PARTLY]],
        'truncs': [TRUNC_IN_IMAGE],
        'amtOccs': [[-1, -1]],
        'amtBorders': [[-1, -1, -1]],
    }
]

def generate_frame_list(output_frame_list, num_lines):
    with open(output_frame_list, 'w') as f:
        for i in range(num_lines):
            f.write(f"{i} {str(i).zfill(6)}\n")

# output_file = 'tracklet_labels.xml'
# create_tracklet_xml(detections, output_file)
