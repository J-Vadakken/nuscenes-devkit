import json

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

CLASS_TO_LABEL_ID = {
    "traffic_sign": 0,
    "yield_sign": 1,
    "stop_sign": 2,
    "railroad_sign": 3,
    "railroad_light": 4,
    "railroad_bar_down": 5,
    "railroad_bar_moving": 6,
    "railroad_bar_up": 7,
    "type_3_barricade": 8,
    "construction_barrel": 9,
    "deer": 10,
    "pedestrian": 11,
    "car": 12,
    "fire_hydrant": 13,
}

# Define the fixed categories
CATEGORIES = [
    {"name": "traffic_sign", "parent": "", "attributes": []},
    {"name": "yield_sign", "parent": "", "attributes": []},
    {"name": "stop_sign", "parent": "", "attributes": []},
    {"name": "railroad_sign", "parent": "", "attributes": []},
    {"name": "railroad_light", "parent": "", "attributes": []},
    {"name": "railroad_bar_down", "parent": "", "attributes": []},
    {"name": "railroad_bar_moving", "parent": "", "attributes": []},
    {"name": "railroad_bar_up", "parent": "", "attributes": []},
    {"name": "type_3_barricade", "parent": "", "attributes": []},
    {"name": "construction_barrel", "parent": "", "attributes": []},
    {"name": "deer", "parent": "", "attributes": []},
    {"name": "pedestrian", "parent": "", "attributes": []},
    {"name": "car", "parent": "", "attributes": []},
    {"name": "fire_hydrant", "parent": "", "attributes": []},
]

ATTRIBUTES = ["occluded"]

def generate_annotation(frame_id, tracks, prev_tracks):
    """
    Generates a JSON annotation for a given frame.
    
    Parameters:
    - frame_id: The ID of the frame.
    - tracks: A list of dictionaries where each dictionary contains the track information for an object.
    
    Returns:
    - A dictionary representing the annotation for the frame.
    """
    annotations = []
    for i, track in tracks.items():
        xyz = list(track['xyz'])
        xyz[-1] -= 1.7
        annotation = {
            "id": i,
            "type": "cuboid_3d",
            "attributes": {
                "occluded": False,
                "track_id": i,
                "keyframe": True,
                "outside": False,
            },
            "group": 0,
            "label_id": CLASS_TO_LABEL_ID[TYPE_TO_CVAT_CLASS[track['type']]],
            "position": xyz,
            "rotation": [0.0, 0.0, track['yaw']],
            "scale": track['lwh'],
        }
        annotations.append(annotation)
    
    # check if any tracks have died
    dead_tracks_ids = set(prev_tracks.keys()) - set(tracks.keys())
    # dead_tracks = [prev_tracks[track_id] for track_id in dead_tracks_ids]
    dead_tracks = {track_id: prev_tracks[track_id] for track_id in dead_tracks_ids}

    for i, dead_track in dead_tracks.items():
        # input(f"{i} has switched to outside")
        xyz = list(dead_track['xyz'])
        xyz[-1] -= 1.7
        annotation = {
            "id": i,
            "type": "cuboid_3d",
            "attributes": {
                "occluded": False,
                "track_id": i,
                "keyframe": True,
                "outside": True,
            },
            "group": 0,
            "label_id": CLASS_TO_LABEL_ID[TYPE_TO_CVAT_CLASS[dead_track['type']]],
            "position": xyz,
            "rotation": [0.0, 0.0, dead_track['yaw']],
            "scale": dead_track['lwh'],
        }
        annotations.append(annotation)
    
    return {
        "id": str(frame_id).zfill(6),
        "annotations": annotations,
        "attr": {
            "frame": frame_id
        },
        "point_cloud": {
            "path": ""
        },
        "media": {
            "path": ""
        }
    }

def generate_datumaro_annotations(annotations):
    """
    Generates annotations for multiple frames.
    
    Parameters:
    - frames: A list of dictionaries, each containing 'frame_id' and 'tracks' information.
    
    Returns:
    - A JSON object with the complete annotation data.
    """
    
    json_data = {
        "info": {},
        "categories": {
            "label": {
                "labels": CATEGORIES,
                "attributes": ATTRIBUTES
            },
            "points": {
                "items": []
            }
        },
        "items": annotations
    }
    
    return json_data