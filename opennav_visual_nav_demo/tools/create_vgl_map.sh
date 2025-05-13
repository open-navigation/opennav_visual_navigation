#!/bin/bash

# Usage: ./create_map.sh /path/to/bag /path/to/output/folder
# Example: ./create_map.sh /mnt/nova_ssd/recordings/data.bag /mnt/nova_ssd/maps

# Check if all arguments are provided
if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ]; then
  echo "Usage: $0 <PATH_TO_SENSOR_DATA_BAG>  <PATH_TO_POSE_DATA_BAG> <PATH_TO_OUTPUT_FOLDER>"
  exit 1
fi

SENSOR_BAG_PATH="$1"
POSE_BAG_PATH="$2"
OUTPUT_FOLDER="$3"

ros2 run isaac_mapping_ros run_rosbag_to_mapping_data.py \
  --sensor_data_bag=$SENSOR_BAG_PATH --pose_bag=$POSE_BAG_PATH --output_folder="$OUTPUT_FOLDER/keyframes" \
  --extract_feature --rot_dist=5 --trans_dist=0.2 # \
  # --pose_topic_name /visual_localization/pose

ros2 run isaac_mapping_ros create_cuvgl_map.py --map_folder=$OUTPUT_FOLDER --no-extract_feature
