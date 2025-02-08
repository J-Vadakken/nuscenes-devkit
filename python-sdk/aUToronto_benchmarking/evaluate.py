import sys
import os
import json

# Get the directory of the current file
current_dir = os.path.dirname(os.path.abspath(__file__))

config_path = os.path.join(current_dir, 'config.json')
class_names_path = os.path.join(current_dir, 'class_names.json')

# Load the config.json file
with open(config_path, 'r') as config_file:
    config = json.load(config_file)

# Add the path to the nuscenes module. This allows for imports
sys.path.append(config["python_sdk_path_"])
from nuscenes.eval.tracking.data_classes import TrackingConfig
from nuscenes.eval.tracking.evaluate import TrackingEval

if __name__ == "__main__":

    result_path_ = os.path.join(config['dataroot_'], 'tracks/track.json')
    output_dir_ = os.path.join(config['dataroot_'], 'output')
    gtpath_ = os.path.join(config['dataroot_'], "gt")

    class_names_path = os.path.join(current_dir, 'class_names.json')
    render_curves_ = config['render_curves_']
    verbose_ = config['verbose_']
    render_classes_ = config['render_classes_']

    eval_set_ = config['eval_set_']
    version_ = config['version_']

    with open(class_names_path, 'r') as _f:
        cfg_ = TrackingConfig.deserialize(json.load(_f))

    nusc_eval = TrackingEval(config=cfg_, result_path=result_path_, eval_set=eval_set_, output_dir=output_dir_,
                             nusc_version=version_, nusc_dataroot=gtpath_, verbose=verbose_,
                             render_classes=render_classes_)
    nusc_eval.main(render_curves=render_curves_)

    print("Evaluation complete.")
    print("Results saved in {}".format(output_dir_))

