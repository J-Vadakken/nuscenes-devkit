# This file takes in a nuscenes dataset, and creates theoretical perfect tracks for each object in the dataset.
# However, note that we set velocity to an aribitrary value of [1.0, 1.0] for each object, as this is moreso 
# just a sanity check.

import json

# Path to the sample_annotation.json, category.json and instance.json files in nuscenes
default_json_path = '/home/jgv555/CS/aUToronto/Nuscenes_Track_Metrics/Our_Data/trial1/Perfect_tracks_2/mini_data/v1.0-mini/sample_annotation.json'
category_file_path = '/home/jgv555/CS/aUToronto/Nuscenes_Track_Metrics/Our_Data/trial1/Perfect_tracks_2/mini_data/v1.0-mini/category.json'
instance_file_path = '/home/jgv555/CS/aUToronto/Nuscenes_Track_Metrics/Our_Data/trial1/Perfect_tracks_2/mini_data/v1.0-mini/instance.json'

output_file_path = '/home/jgv555/CS/aUToronto/Nuscenes_Track_Metrics/nuscenes-devkit/Data/tracks/track.json'

# Load the default.json file
with open(default_json_path, 'r') as f:
    default = json.load(f)

with open(category_file_path, 'r') as f:
    category = json.load(f)
cat_to_name = {}
for thing in category:
    cat_to_name[thing["token"]] = thing["name"]

with open(instance_file_path, 'r') as f:
    instance = json.load(f)
samp_to_name = {}
for thing in instance:
    tok = thing["category_token"]
    samp_to_name[thing["token"]] = cat_to_name[tok]


# Load timestamp_dict
# with open(timestamp_file_path, 'r') as f:
#     timestamp_dict = json.load(f)

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

# associatiing previous tracked objects with newer ones for velocity
track_connections = {}
# Will have this format: {track_id: []}
# Where [] is a list of lists, where each entry is 
# [frame_num, index]

for entry in default:

    frame_token = entry["sample_token"]    
    sample_result = {
        "sample_token":   entry["sample_token"],                              #<str>         -- Foreign key. Identifies the sample/keyframe for which objects are detected.
        "translation":    entry["translation"],                        #<float> [3]   -- Estimated bounding box location in meters in the global frame: center_x, center_y, center_z.
        "size":           entry["size"],                           #<float> [3]   -- Estimated bounding box size in meters: width, length, height.
        "rotation":       entry["rotation"],                              #<float> [4]   -- Estimated bounding box orientation as quaternion in the global frame: w, x, y, z.
        "velocity":       [1.0, 1.0],                                 #<float> [2]   -- Estimated bounding box velocity in m/s in the global frame: vx, vy.
        "tracking_id":    entry["instance_token"],     #<str>         -- Unique object id that is used to identify an object track across samples.
        "tracking_name":  samp_to_name[entry["instance_token"]],  #<str>         -- The predicted class for this sample_result, e.g. car, pedestrian.
                                                                    #              Note that the tracking_name cannot change throughout a track.
        "tracking_score": 1                                         #<float>       -- Object prediction score between 0 and 1 for the class identified by tracking_name.
                                                                    #              We average over frame level scores to compute the track level score.
                                                                    #              The score is used to determine positive and negative tracks via thresholding.
    }
    if results.get(frame_token) == None:
        results[frame_token] = []
    results[frame_token].append(sample_result)


# Last thing
submission["results"] = results
write_json(submission)

print("New track file created at: ", output_file_path)