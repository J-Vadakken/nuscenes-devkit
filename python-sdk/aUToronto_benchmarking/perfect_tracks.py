# This file takes in a Datumaro dataset, and creates 
# theoretical perfect tracks for each object in the dataset.

# You MUST have a timestamp dictionary already created
# You MUST have a default.json file that contains the CVAT annotations in the Datumaro 3D 1.0 format
# In the config file, make sure you've linked the paths properly
# This code will create the tracks in the nuscenes submission format, and store it in 
# dataroot_ + 'track/track.json'

# IMPORTANT: if you already have another track in that folder, that you wish to save, either rename it
# or be preparted to lose it. 

# This file takes in a Datumaro dataset, and creates theoretical perfect tracks for each object in the dataset.


import json
import os
import numpy as np

# Get the directory of the current file
current_dir = os.path.dirname(os.path.abspath(__file__))

config_path = os.path.join(current_dir, 'config.json')

# Load the config.json file
with open(config_path, 'r') as config_file:
    config = json.load(config_file)

# Path to the default.json file
default_json_path = config["datumaro_dataroot_"]
timestamp_file_path = config["dataroot_"] + '/timestamp'
output_file_path = config["dataroot_"] + '/tracks/track.json'

# Load the default.json files
json_files = []
for filename in sorted(os.listdir(default_json_path)):
    if filename.endswith('.json'):
        with open(os.path.join(default_json_path, filename), 'r') as file:
            json_files.append(json.load(file))
default = json_files[0]

# Load timestamp_dict
timestamp_files = []
for filename in sorted(os.listdir(timestamp_file_path)):
    if filename.endswith('.json'):
        with open(os.path.join(timestamp_file_path, filename), 'r') as file:
            timestamp_files.append(json.load(file))
timestamp_dict = timestamp_files[0]

if not os.path.exists(config["dataroot_"] + '/tracks'):
    os.makedirs(config["dataroot_"] + '/tracks')

# Create a function to easily write to json files
def write_json(data):
    with open(output_file_path, 'w') as file:
        json.dump(data, file, indent=4)

# Output format:
submission = {
    "meta": {
        "use_camera":   False, #<bool>  -- Whether this submission uses camera data as an input.
        "use_lidar":    False, #<bool>  -- Whether this submission uses lidar data as an input.
        "use_radar":    False, #<bool>  -- Whether this submission uses radar data as an input.
        "use_map":      False, #<bool>  -- Whether this submission uses map data as an input.
        "use_external": False, #<bool>  -- Whether this submission uses external data as an input.
    },
    "results" : {}
}
results = {}
# Creating tokens for sample_data
def sample_data_token(frame_number):
    return str(f"{frame_number:032d}")

# Finding time
def find_time(time_list):
    time_val1 = time_list[0] * 10**6
    time_val2 = time_list[1] // (10**3)
    time_val = time_val1 + time_val2
    return time_val

# Creating dictionary for track_id to category name
track_id_to_category = {}
for i, item in enumerate(default["categories"]["label"]["labels"]):
    track_id_to_category[i] = item["name"]

# associatiing previous tracked objects with newer ones for velocity
track_connections = {}
# Will have this format: {track_id: []}
# Where [] is a list of lists, where each entry is 
# [frame_num, index]

# Creating a token for each frame
for thing in default["items"]:
    frame_num = thing["attr"]["frame"]
    frame_token = sample_data_token(frame_num)
    results[frame_token] = []
    for j, annot in enumerate(thing["annotations"]):
        rotation_3d = annot["rotation"]
        rotation_4d = [rotation_3d[2]] + [0,0,1]

        # For velocity
        if not(annot["attributes"]["track_id"] in track_connections):
            track_connections[annot["attributes"]["track_id"]] = [[frame_num, j]]
            vel = [np.nan, np.nan]
        else:
            track_connections[annot["attributes"]["track_id"]].append([frame_num, j])
            prev_frame = track_connections[annot["attributes"]["track_id"]][-2][0]
            prev_ind = track_connections[annot["attributes"]["track_id"]][-2][1]
            prev_annot = results[sample_data_token(prev_frame)][prev_ind]
            # Hard code
            if timestamp_dict.get(str(frame_num)) == None:
                vel = [np.nan, np.nan]
            else:
                prev_time = find_time(timestamp_dict[str(prev_frame)])
                cur_time = find_time(timestamp_dict[str(frame_num)])
                prev_loc = prev_annot["translation"]
                cur_loc = annot["position"]
                vx = (cur_loc[0] - prev_loc[0]) / (cur_time - prev_time)
                vy = (cur_loc[1] - prev_loc[1]) / (cur_time - prev_time)
                vel = [vx, vy]



        sample_result = {
            "sample_token":   frame_token,                              #<str>         -- Foreign key. Identifies the sample/keyframe for which objects are detected.
            "translation":    annot["position"],                        #<float> [3]   -- Estimated bounding box location in meters in the global frame: center_x, center_y, center_z.
            "size":           annot["scale"],                           #<float> [3]   -- Estimated bounding box size in meters: width, length, height.
            "rotation":       rotation_4d,                              #<float> [4]   -- Estimated bounding box orientation as quaternion in the global frame: w, x, y, z.
            "velocity":       vel,                                 #<float> [2]   -- Estimated bounding box velocity in m/s in the global frame: vx, vy.
            "tracking_id":    str(annot["attributes"]["track_id"]),     #<str>         -- Unique object id that is used to identify an object track across samples.
            "tracking_name":  track_id_to_category[annot["label_id"]],  #<str>         -- The predicted class for this sample_result, e.g. car, pedestrian.
                                                                        #              Note that the tracking_name cannot change throughout a track.
            "tracking_score": 1                                         #<float>       -- Object prediction score between 0 and 1 for the class identified by tracking_name.
                                                                        #              We average over frame level scores to compute the track level score.
                                                                        #              The score is used to determine positive and negative tracks via thresholding.
        }
        results[frame_token].append(sample_result)


# Last thing
submission["results"] = results
write_json(submission)

print("New track file created at: ", output_file_path)