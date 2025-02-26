# this file reads kitti format xml file and also timestamp_dict.json to generate a json file in custom format

import os
import json
import xml.etree.ElementTree as ET
import numpy as np

if __name__ == "__main__":
    # read timestamp_list.json
    with open('annotations/timestamp_dict_2.json', 'r') as f:
        timestamp_dict = json.load(f)

    # data should be a list of dictionaries
    # each dictionary should have the following
    # 'timestamp': timestamp
    # 'frame': frame number
    # 'objects': list of objects
    # each object should have the following
    # 'type': object type
    # 'center': [x, y, z]
    # 'size': [l, w, h]
    # 'yaw': yaw angle
    
    # create a dictionary of dictionaries, and later process it to a list of dictionaries
    data = {}
    for key in timestamp_dict:
        data[key] = {}
        data[key]['timestamp'] = timestamp_dict[key]
        data[key]['frame'] = key
        data[key]['objects'] = []
    # 

    # read kitti format xml file
    tree = ET.parse('tracklet_labels_06_06.xml')
    root = tree.getroot()
    tracklets = root.find('tracklets')
    items = tracklets.findall('item')

    for item in items:
        object_type = item.find('objectType').text
        object_type = object_type.replace(" ", "_").lower()
        poses = item.find('poses')
        
        for pose in poses.findall('item'):
            frame_id = int(item.find('first_frame').text)
            if str(frame_id) not in data:
                continue
            x = float(pose.find('tx').text)
            y = float(pose.find('ty').text)
            z = float(pose.find('tz').text)
            dz = float(item.find('h').text)
            dy = float(item.find('w').text)
            dx = float(item.find('l').text)
            heading_angle = float(pose.find('rz').text)

            data[str(frame_id)]['objects'].append({
                'type': object_type,
                'center': [x, y, z],
                'size': [dx, dy, dz],
                'yaw': heading_angle
            })

    # convert dictionary of dictionaries to list of dictionaries
    data = list(data.values())


    # sort data by timestamp
    data.sort(key=lambda x: x['timestamp'])

    # write to json file
    with open('cvat_06-06.json', 'w') as f:
        json.dump(data, f)