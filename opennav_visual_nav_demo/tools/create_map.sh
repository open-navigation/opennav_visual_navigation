#!/bin/bash

# Usage: ./create_map.sh /path/to/bag /path/to/output/folder
# Example: ./create_map.sh /mnt/nova_ssd/recordings/data.bag /mnt/nova_ssd/maps

# Check if both arguments are provided
if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Usage: $0 <PATH_TO_BAG> <PATH_TO_OUTPUT_FOLDER>"
  exit 1
fi

BAG_PATH="$1"
OUTPUT_FOLDER="$2"

# Run the ROS 2 command
ros2 run isaac_mapping_ros create_map_offline.py --sensor_data_bag="$BAG_PATH" --base_output_folder="$OUTPUT_FOLDER"
