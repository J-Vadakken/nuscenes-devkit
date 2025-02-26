# Extraction scripts to export tracker boxes to CVAT for faster labelling

`message_definitions.py`: defines the radar, and 3D tracked box message types

`process_bag.py`: reads the rosbag and exports it
* Specify rosbag_dir in the file
* Can toggle `plot_bev_point_cloud_and_images` for visualizations

`create_kitti_annotations.py`: helper functions to export in kitti format

`create_datumaro_annotations.py`: helper functions to export in datumaro format


## `process_bag.py` instructions

`python3 process_bag.py`
Uncomment `plot_bev_point_cloud_and_images(...)` to enable local visualizations of boxes.
To export annotations make a new folder, move the annotations folder (containing `default.json`) into it and zip it.
Upload on CVAT choosing the DATUMURO format