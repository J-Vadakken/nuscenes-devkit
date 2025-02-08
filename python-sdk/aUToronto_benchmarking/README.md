# Purpose
This code has been modified from the nuscenes team. These scripts can be used to evaluate for dynamic benchmarking of the aUToronto tracker outputs.

Using ground truths from cvat, we export the data in the Datumaro format. We record our tracker's outputs and measure it against the ground truths.

The how to us direction will include step by step instructions

# Table of Contents
- [How to use](#how-to-use)
- [What each file does](#what-each-file-does)
- [Installing Dependencies](#installing-dependencies)
- [Metrics](#metrics)


# How to use
## Requirements

1. The Rosbag
2. Cvat annotations in Datumaro 3D 1.0 format.
3. Tracker outputs in the specified nuscenes format (required for benchmarking, but you can still run the code without it)
4. Python library: motmetrics<=1.1.3 
5. Python library pandas>=0.24

See [Installing Dependencies](#installing-dependencies) for installing these libraries.


## Directives for usage
1. Download the **Datumaro dataset** in some location.
2. Download the associated **rosbag**.
3. **Create a directory** for holding the data that will be converted to the nuscenes format. Let the directory location be called **dataroot_**
4. **Update the config.json** file with relevant file locations. See [What each file does](#what-each-file-does)
5. Use **timestamp_gen.py** to create timestamps.
6. Use **gt_conversions.py** to create the ground truths.
7. **Store your tracker outputs** in dataroot_/track/track.json.
8. (Alt to 7) Use **perfect_track.py** to create perfect tracker outputs based on the gts. Note that this will clear the track/track.json file.
9. **Run evaluate.py** to evaluate tracker. Outputs will be stored in dataroot_/outputs

# File Descriptions
## config.json
Contains all the file paths. You will need to update this to make it work for your computer. 

- **python_sdk_path_** : This is to make it easier for the code to import modules. Set it equal to the absolute containg directory that ends with **"python-sdk"**
- **datumaro_dataroot_** : Set it equal to the **json file** that contains the exported CVAT annotations in the Datumaro 3D 1.0 format.
- **rosbag_path_"** : The path to the rosbag
- **dataroot_"** : The path to the folder you want your nuscenes-formatted data to be stored in. Will include outputs.
- **eval_set_"** Don't touch!
- **version_"** Don't touch!
- **render_curves_"**: Whether you want to generate graphs or not. I receommend leaving it on.
- **verbose_"**: Just leave it as default
- **render_classes_"**: Just leave this as default

## class_names.json
Contains all the class names for our tracker. Make sure to update it appropriatly if there are any changes. 

## timestamp_gen.py
**Creates** the timestamp dictionary.

**Requires** a rosbag file path (which should be in the config.json file). **Outputs** a frame-to-timestamp dictionary json file. This will be found in dataroot_\\timestamp_dict.json

Run this script first! It will some time to run.

## perfect_tracks.py
**Creates** perfect sample tracker outputs.

**Requires** the timestamp dicionary created with timestamp_gen.py. **Requires** the Datumaro 3D annotations. **Outputs** theoretical perfect tracker outputs based on the provided ground truths. This will be stores in dataroot_\\track\\track.json. 

Note that all whatever tracker output you wish to be evaluates must be stored in dataroot_\\track\\track.json. Running this code will wipe whatevver previous values were in this file so be careful!

## gt_conversion.py
**Creates** the ground truths in the nuscenes format.

**Requires** the timestamp dicionary created with timestamp_gen.py. **Requires** the Datumaro 3D annotations. **Outputs** ground truth values in the nuscenes format. These will be stored in dataroot_\\gt.

## evaluate.py
**Evaluates** the tracker's tracks against the ground truths, and outputs the results.

**Requires** the ground truths, as created by gt_conversion.py. **Requires** Tracks as created by either perfect_tracks.py or by importing it manually. **Outputs** the results of the tracker in dataroot_\\output.

For a description of the metrics used, see [Metrics](#metrics)

# Installing Dependencies

Installing Pandas should be easy. 

You can install the correct version of motmetrics with the following line of code: 

```
    pip install motmetrics==1.1.3
```
If you already have it installed, make sure it is the correct version.

Unfortunately, version 1.1.3 of motmetrics is imcompatible with Python 3.10.
You will get the following error when you try to run the code: 
````
  File "/home/jgv555/CS/aUToronto/Nuscenes_Track_Metrics/nuscenes-devkit/python-sdk/nuscenes/eval/tracking/mot.py", line 13, in <module>
    import motmetrics
  File "/home/jgv555/.local/lib/python3.10/site-packages/motmetrics/__init__.py", line 4, in <module>
    import motmetrics.metrics
  File "/home/jgv555/.local/lib/python3.10/site-packages/motmetrics/metrics.py", line 8, in <module>
    from collections import OrderedDict, Iterable
ImportError: cannot import name 'Iterable' from 'collections' (/usr/lib/python3.10/collections/__init__.py)
````
To debug this, Ctrl+Click on the bottom most file that gives you this error. Or find this file and open it in your editor.

Find this line (Should be line 8)
````
from collections import OrderedDict, Iterable
````
And change it to
````
from collections import OrderedDict
from collections.abc import Iterable
````

The error occurs because Iterable has been moved from collections to collections.abc in Python 3.10. To fix this error, you need to update the import statement in the motmetrics library to import Iterable from collections.abc.

# Metrics

## Summarizing Table

| Metric  | Description  | Formula  | Range  |
|---------|-------------|----------|--------|
| **recall** | Measures the proportion of true positive detections out of the total number of ground truth objects. | recall = tp / (tp + fn) | [0, 1] |
| **motar** (Multiple Object Tracking Accuracy with Recall) | A variant of MOTA that incorporates recall, measuring tracking accuracy while considering recall. | motar = (tp - fp - ids) / (tp + fn) | [0, 1] |
| **mota** (Multiple Object Tracking Accuracy) | Measures tracking accuracy by considering false positives, false negatives, and identity switches. | mota = 1 - (fp + fn + ids) / gt | [-∞, 1] |
| **motp** (Multiple Object Tracking Precision) | Measures tracking precision by calculating the average distance between ground truth and predicted positions. | motp = ∑d / tp (where d is the distance between ground truth and predicted positions) | [0, ∞] |
| **mt** (Mostly Tracked) | The number of ground truth objects successfully tracked for at least 80% of their lifespan. | N/A | [0, gt] |
| **ml** (Mostly Lost) | The number of ground truth objects tracked for less than 20% of their lifespan. | N/A | [0, gt] |
| **faf** (False Alarms per Frame) | The average number of false positive detections per frame. | faf = fp / num_frames | [0, ∞] |
| **gt** (Ground Truth) | The total number of ground truth objects in the dataset. | N/A | [0, ∞] |
| **tp** (True Positives) | The number of correctly detected objects. | N/A | [0, ∞] |
| **fp** (False Positives) | The number of incorrectly detected objects. | N/A | [0, ∞] |
| **fn** (False Negatives) | The number of missed ground truth objects. | N/A | [0, ∞] |
| **ids** (Identity Switches) | The number of times an object is assigned a new identity. | N/A | [0, ∞] |
| **frag** (Fragmentations) | The number of times a ground truth object is interrupted in the tracking sequence. | N/A | [0, ∞] |
| **tid** (Track Initialization Delay) | The average delay in initializing tracks. | N/A | [0, ∞] |
| **lgd** (Longest Gap Duration) | The longest gap in the tracking sequence for any object. | N/A | [0, ∞] |
| **amota** (Average Multiple Object Tracking Accuracy) | The average MOTA over different recall thresholds. | N/A | [-∞, 1] |
| **amotp** (Average Multiple Object Tracking Precision) | The average MOTP over different recall thresholds. | N/A | [0, ∞] |

| Metric  | Perfect Value | Explanation (Perfect) | Worst Value | Explanation (Worst) |
|---------|--------------|-----------------------|-------------|---------------------|
| **recall** | 1.0 | All ground truth objects are detected (no false negatives). | 0.0 | No ground truth objects are detected (all are false negatives). |
| **motar** | 1.0 | All true positives are correctly tracked, with no false positives or identity switches. | -∞ | Only false positives and identity switches, with no true positives. |
| **mota** | 1.0 | No false positives, false negatives, or identity switches. | -∞ | Tracking fails completely—only false positives, false negatives, and identity switches exist. |
| **motp** | 0.0 | Predicted positions exactly match ground truth (zero distance error). | ∞ | Predictions are infinitely far from ground truth. |
| **mt** (Mostly Tracked) | gt | All ground truth objects are tracked for at least 80% of their lifespan. | 0 | No ground truth objects are successfully tracked for 80% of their lifespan. |
| **ml** (Mostly Lost) | 0 | No ground truth objects are lost for more than 20% of their lifespan. | gt | All ground truth objects are lost for more than 20% of their lifespan. |
| **faf** (False Alarms per Frame) | 0.0 | No false positive detections per frame. | ∞ | Every frame is filled with false positives. |
| **gt** (Ground Truth) | N/A | The dataset's total number of objects (unchanged). | N/A | The dataset's total number of objects (unchanged). |
| **tp** (True Positives) | gt | All ground truth objects are correctly detected. | 0 | No true positives—all detections are false positives or missing. |
| **fp** (False Positives) | 0 | No incorrect detections. | ∞ | Every detection is incorrect. |
| **fn** (False Negatives) | 0 | No missed detections. | gt | All ground truth objects are missed. |
| **ids** (Identity Switches) | 0 | No identity switches occur. | ∞ | Every object frequently changes identity. |
| **frag** (Fragmentations) | 0 | No interruptions in object tracking sequences. | ∞ | Every object’s tracking sequence is frequently interrupted. |
| **tid** (Track Initialization Delay) | 0 | No delay in track initialization. | ∞ | Tracks take infinitely long to initialize. |
| **lgd** (Longest Gap Duration) | 0 | No gaps in tracking any object. | ∞ | The longest tracking gap is infinite (objects disappear permanently). |
| **amota** | 1.0 | The average MOTA over different recall thresholds is perfect. | -∞ | The worst possible tracking accuracy across all recall thresholds. |
| **amotp** | 0.0 | The average MOTP over different recall thresholds is perfect (zero localization error). | ∞ | The worst possible tracking precision—objects are detected at infinitely incorrect positions. |
