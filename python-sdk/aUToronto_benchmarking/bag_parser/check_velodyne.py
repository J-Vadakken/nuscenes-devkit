from rosbags.highlevel import AnyReader
from pathlib import Path

ALL_BAGS_DIR = '~/bags/'
VELODYNE_TOPIC = '/velodyne_points'
VELODYNE_BAGS = {}
VERBOSE = False

if __name__ == '__main__':
    # Resolve the directory path
    rosbag_dir = Path(ALL_BAGS_DIR).expanduser()
    
    if not rosbag_dir.exists():
        print(f"Directory {rosbag_dir} does not exist.")
    else:
        # Recursively find all subfolders with `.db3` files
        for subfolder in rosbag_dir.glob('**/'):  # Recursive glob for subfolders
            yaml_files = list(subfolder.glob('*.yaml'))
            db3_files = list(subfolder.glob('*.db3'))

            # Ensure the subfolder contains both a `.yaml` and a `.db3` file
            if yaml_files and db3_files:
                print(f"Processing folder: {subfolder}")
                try:
                    with AnyReader([subfolder]) as reader:
                        connections = [(x.topic, x.msgcount) for x in reader.connections if x.topic == VELODYNE_TOPIC]
                        if connections:
                            if VERBOSE:
                                print(f"VELODYNE_TOPIC found in {subfolder}\n")
                            VELODYNE_BAGS[subfolder] = connections[0][1]
                except Exception as e:
                    if VERBOSE:
                        print(f"Error reading {subfolder}: {e}")
    
    VELODYNE_BAGS = dict(sorted(VELODYNE_BAGS.items(), key=lambda x: x[1], reverse=True))
    print("\n ================== SUMMARY ================== \n")
    print(f'Found {len(VELODYNE_BAGS)} bags with {VELODYNE_TOPIC}')
    print('Printing from most number messages to least number of messages:\n')
    for bag, count in VELODYNE_BAGS.items():
        print(f"{bag}: {count} messages")
        
    # Then save bag directories to a text file
    with open('velodyne_bags.txt', 'w') as f:
        for bag in VELODYNE_BAGS.keys():
            f.write(f"'{bag}'\n")
    print("\n ================== END SUMMARY ================== \n")
    print("\nSaved bag directories to 'velodyne_bags.txt' in the current directory.")