# This script is used to create frame to timstamp dictionary for use in conversions. 

# You MUST have a rosbag to use this script, and make sure the path to it is correct in the config.json file.
# This script will store the created dicionary in dataroot_ + 'timestamp_dict.json' where dataroot_ is from the config file.

# This script just calls the process_bag2.py script, which is the one that actually does the work.
# proccess_bag2.py can be found in nuscenes-devkit/python-sdk/aUToronto_benchmarking/bag_parser/process_bag2.py
# If you want to see the outputs in real time, you can directly run that script instead. 
# However, I leave this here just for convenience.

import subprocess

def execute_script():
    script_path = '/home/jgv555/CS/aUToronto/Nuscenes_Track_Metrics/nuscenes-devkit/python-sdk/aUToronto_benchmarking/bag_parser/process_bag2.py'
    result = subprocess.run(['python3', script_path], capture_output=True, text=True)
    print("Output:", result.stdout)
    if result.stderr == '':
        print("Error: None")
    else:
        print("Error:", result.stderr)

if __name__ == "__main__":
    execute_script()