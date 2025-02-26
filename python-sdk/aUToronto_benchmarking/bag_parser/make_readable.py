import json
file_path = "/home/jgv555/CS/aUToronto/Cvat_extraction_2/bag_parser/annotations/default.json"
output_path = "/home/jgv555/CS/aUToronto/Cvat_extraction_2/bag_parser/annotations/default_readable.json"

with open(file_path, "r") as file:
    data = json.load(file)

with open(output_path, "w") as file:
    json.dump(data, file, indent=4)