# This file takes in a Datumaro dataset, and creates the ground truths for the nuscenes evaluation code. 

# You MUST have a timestamp dictionary already created
# You MUST have a default.json file that contains the CVAT annotations in the Datumaro 3D 1.0 format
# In the config file, make sure you've linked the paths properly
# This code will create the ground truths in the nuscenes format, and store it in 
# dataroot_ + 'gt/v1.0-mini'


import sys
import os
import json
import uuid

# Get the directory of the current file
current_dir = os.path.dirname(os.path.abspath(__file__))

config_path = os.path.join(current_dir, 'config.json')

# Load the config.json file
with open(config_path, 'r') as config_file:
    config = json.load(config_file)

# Some assumptions we've made for this code:
# For sake of coordination between results and the original data, we will use the frame number
# to generate the sample_token for each frame, by extending the frame number to a 32-character


# Path to the default.json file
default_json_path = config["datumaro_dataroot_"]
timestamp_dict_json_path = config["dataroot_"] + '/timestamp_dict.json' # Path to the timestamp dictionary
nusc_folder_path = config["dataroot_"] + '/gt/v1.0-mini' # Path to where you want to output the ground truths. 

# Path to parent point cloud folder
#pcd_folder_path = config["datamaro_dataroot_pcd_"]

# Read the default.json file
with open(default_json_path, 'r') as file:
    default_data = json.load(file)
# Red the timestamps file
with open(timestamp_dict_json_path, 'r') as file:
    timestamp_dict = json.load(file)
# Ensure the nusc folder exists
os.makedirs(nusc_folder_path, exist_ok=True)

# Create dictionary mapping to 13 json files
nusc_json_files = {}
nusc_file_names = ["attribute", "log" , "scene",
    "calibrated_sensor", "map" , "sensor", "category" , 
    "sample_annotation", "visibility", "ego_pose" , "sample_data",
    "instance" , "sample"
]
# Write 13 new JSON files
for file_name in nusc_file_names:
    new_file_path = os.path.join(nusc_folder_path, f"{file_name}.json")
    nusc_json_files[file_name] = new_file_path

    # with open(new_file_path, 'w') as new_file:
    #     json.dump(default_data, new_file, indent=4)

# Create a function to easily write to json files
def write_json(file_name, data):
    with open(nusc_json_files[file_name], 'w') as file:
        json.dump(data, file, indent=4)

print("13 new JSON files have been created in the nusc folder.")

# For creating unique tokens
def unique_token_generator():
    return uuid.uuid4().hex

# Creating tokens for sample_data
def sample_data_token(frame_number):
    return str(f"{frame_number:032d}")


# JSON files:
# Creating some tokens for universal use.

# For the categories
# class_categories = ["traffic_sign","yield_sign","stop_sign",
#                     "railroad_bar_down","type_3_barricade",
#                     "construction_barrel","deer","pedestrian",
#                     "car","other_sign"]
category_tokens = {}
num_to_category = {}
class_categories = []
# class categories by numbers
for i, category in enumerate(default_data["categories"]["label"]["labels"]):
    num_to_category[i] = category["name"]
    class_categories.append(category["name"])
for i, category in enumerate(class_categories):
    category_tokens[category] = unique_token_generator()


# For scene
scene_token = unique_token_generator()

# For log
log_token = unique_token_generator()

# For sample.json, which keeps tracks of frames
# print(len(default_data["items"]))
# min_num = -1
# fl = []
# for a in default_data["items"]:
#     if min_num + 1 != a["attr"]["frame"]:
#         fl.append(min_num + 1)
#     min_num = a["attr"]["frame"]
# default.json for job 256 is missing frames: [116, 128, 260, 309, 416]

# For sample_data. Must make sure that timestamp and frame_number actually match.
sample_data_tokens = {}
valid_frames = []
for i, sample_data in enumerate(default_data["items"]):
    #sample_data_tokens[sample_data["attr"]["frame"]] = unique_token_generator()
    # more convenient to use index
    if timestamp_dict.get(str(default_data["items"][i]["attr"]["frame"])) == None:
        print("Frame ", default_data["items"][i]["attr"]["frame"], " is missing")
        continue
    valid_frames.append(i)
    sample_data_tokens[i] = sample_data_token(default_data["items"][i]["attr"]["frame"])

# Need an ego_poes token for each timestamp. 
ego_pose_tokens = {}
for i in valid_frames:
    ego_pose_tokens[i] = unique_token_generator()

sensor_token = unique_token_generator()

calibrated_sensor_token = unique_token_generator()

# Need to keep track of each instance of an object. and create annotation tokens.
instance_tokens = {}
first_instance_tokens = {}
last_instance_tokens = {}
nbr_instance_tokens = {}
instance_category_tokens = {}
annotation_tokens = {}
prev_annotation_token = {}
next_annotation_token = {}
for i in range(len(valid_frames)):
    item_ind = valid_frames[i]

    if annotation_tokens.get(item_ind) == None:
        annotation_tokens[item_ind] = {}
    for j, instance in enumerate(default_data["items"][item_ind]["annotations"]):
        annotation_tokens[item_ind][j] = unique_token_generator()
        tid = instance["attributes"]["track_id"]
        if instance_tokens.get(tid) == None:
            instance_tokens[tid] = unique_token_generator()
            first_instance_tokens[tid] = annotation_tokens[item_ind][j]
            last_instance_tokens[tid] = annotation_tokens[item_ind][j]
            nbr_instance_tokens[tid] = 1
            instance_category_tokens[tid] = category_tokens[num_to_category[instance["label_id"]]]
            prev_annotation_token[annotation_tokens[item_ind][j]] = ""
            next_annotation_token[annotation_tokens[item_ind][j]] = ""
        else:
            prev_annotation_token[annotation_tokens[item_ind][j]] = last_instance_tokens[tid]
            next_annotation_token[last_instance_tokens[tid]] = annotation_tokens[item_ind][j]
            next_annotation_token[annotation_tokens[item_ind][j]] = ""
            last_instance_tokens[tid] = annotation_tokens[item_ind][j]
            nbr_instance_tokens[tid] += 1


# First we'll do the bird files

# 1 
# Create the attribute.json file
# We can leave this empty
write_json("attribute", [])

# 2
# calibrated_sensor.json
# I'm not sure if we can leave this empy, but let's try and see where it leads us
# to_write = [{
#     "token": calibrated_sensor_token,
#     "sensor_token": sensor_token,
#     "translation": [ 0, 0, 0],
#     "rotation": [ 0, 0, 0, 0],
#    "camera_intrinsic": []
#     }
# ]
write_json("calibrated_sensor", [])


# 3. sensor.json
# Putting in dummy values here.
# to_write = {
#     "token": sensor_token,
#     "channel": "RADAR",
#     "modality": "radar"
# }
write_json("sensor", [])

# 4 visibility
to_write = [{
    "description": "I literally don't care. Between 0 and a 100",
    "token": "1",
    "level": "v0-40"
}]
write_json("visibility", to_write)

# 5. categories
# category.json
to_write_in = []
for i, category in enumerate(class_categories):
    to_write_in.append({
        "token": category_tokens[category],
        "name": category,
        "description": ""
    })
write_json("category", to_write_in)

# 6. scene.json
to_write = [
    {
    "token": scene_token,
    "log_token": log_token,
    "nbr_samples": len(default_data["items"]),
    "first_sample_token": sample_data_tokens[0],
    "last_sample_token": sample_data_tokens[len(sample_data_tokens) - 1],
    "name": "DA SCENE",
    "description": "STUPID IDIOTIC SCENE"
    }
]
write_json("scene", to_write)

# 7. map.json
write_json("map", []) # Hopefully, leaving it blank to indicate we have no maps is ok
# From analyzing the file it seems fine.

# 8. log.json
to_write = [{
    "token": log_token,
    "logfile": "",
    "vehicle": "ARTEMIS",
    "date_captured": "n.d",
    "location": "Atlantis"
    }]
write_json("log", to_write)

# 9. sample.json
to_write = []

for i, item_num in enumerate(valid_frames):
    #print(i, " ", item_num)
    if i == 0:
        prev = ""
    else:
        prev = sample_data_tokens[i-1]
    if i == len(valid_frames) - 1:
        next = ""
    else:
        next = sample_data_tokens[i+1]

    time_list = timestamp_dict[str(default_data["items"][item_num]["attr"]["frame"])]
    time_val1 = time_list[0] * 10**6
    time_val2 = time_list[1] // (10**3)
    time_val = time_val1 + time_val2
    to_write.append({
    "token": sample_data_tokens[i],
    "timestamp": time_val,
    "prev": prev,
    "next": next,
    "scene_token": scene_token
    })
write_json("sample", to_write)

# 10 ego_pose.json
# As far as I can tell, these values are not used for tracking purposes,
# so we may put in dummy variables
to_write = []
for i in valid_frames:
    to_write.append({
        "token": ego_pose_tokens[i],
        "timestamp": timestamp_dict[str(default_data["items"][i]["attr"]["frame"])],
        "rotation": [0,0,0,0],
        "translation": [0,0,0]
    })
write_json("ego_pose", to_write)



# 11. sample_data.json
to_write = []
for i in range(len(valid_frames)-1):
    if i == 0:
        prev = ""
    else:
        prev = sample_data_tokens[i-1]
    if i == len(valid_frames) - 1:
        next = ""
    else:
        next = sample_data_tokens[i+1]
    item_ind = valid_frames[i]

    time_list = timestamp_dict[str(default_data["items"][item_ind]["attr"]["frame"])]
    time_val1 = time_list[0] * 10**6
    time_val2 = time_list[1] // (10**3)
    time_val = time_val1 + time_val2
    to_write.append({
        "token": ego_pose_tokens[item_ind], # "",
        "sample_token": sample_data_tokens[item_ind], 
        "ego_pose_token": ego_pose_tokens[i], # "",
        "calibrated_sensor_token": "",
        "timestamp": time_val,
        "fileformat": "pcd",
        "is_key_frame": False,
        "height": 0, # Prob not important
        "width": 0, # Prob not important
        "filename": "", #f"{pcd_folder_path}/{item_ind:06d}.pcd",
        "prev": prev,
        "next": next
    })
write_json("sample_data", to_write)

# 12. instance.json
to_write = []
for key in instance_tokens:
    to_write.append({"token": instance_tokens[key],
                    "category_token": instance_category_tokens[key],
                    "nbr_annotations": nbr_instance_tokens[key],
                    "first_annotation_token": first_instance_tokens[key],
                    "last_annotation_token": last_instance_tokens[key]
                    })
write_json("instance", to_write)

# 13. sample_annotation.json
to_write = []
for i in range(len(valid_frames)):
    valid_ind = valid_frames[i]
    for j, instance in enumerate(default_data["items"][valid_ind]["annotations"]):
        
        rotation_3d = instance["rotation"]  # Extract the 3-element rotation vector
        # Assume the axis of rotation is ALWAYS the z-axis.
        rotation_4d = [rotation_3d[2]] + [0, 0, 1] # Convert to 4-element rotation vector
        to_write.append({
            "token": annotation_tokens[valid_ind][j],
            "sample_token": sample_data_tokens[valid_ind],
            "instance_token": instance_tokens[instance["attributes"]["track_id"]],
            "visibility_token": "1",
            "attribute_tokens": [],
            "translation": instance["position"],
            "size": instance["scale"],
            "rotation": rotation_4d,
            "prev": prev_annotation_token[annotation_tokens[valid_ind][j]],
            "next": next_annotation_token[annotation_tokens[valid_ind][j]],
            "num_lidar_pts": 1, # Prob not important. Actually was important for a filter.
            "num_radar_pts": 0 # Prob not important
            })
write_json("sample_annotation", to_write)

# {
# "token": "076a7e3ec6244d3b84e7df5ebcbac637",
# "sample_token": "ca9a282c9e77460f8360f564131a8af5",
# "instance_token": "cfd5f1ab81ff4f81b8dbebf4955563d6",
# "visibility_token": "2",
# "attribute_tokens": [
# "cb5118da1ab342aa947717dc53544259"
# ],
# "translation": [
# 421.971,
# 1233.295,
# 2.351
# ],
# "size": [
# 2.909,
# 6.908,
# 3.558
# ],
# "rotation": [
# 0.8173410335230723,
# 0.0,
# 0.0,
# 0.5761541763446968
# ],
# "prev": "",
# "next": "91937fcf68224fbd97ec4de10ec4e447",
# "num_lidar_pts": 3,
# "num_radar_pts": 2
# }

print(f"New files created in {nusc_folder_path}")
