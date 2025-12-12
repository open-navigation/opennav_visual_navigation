#!/usr/bin/env bash
#
#
#This script launches the full ISAAC ROS Perceptor stack suitable for autonomous navigation with ROS2 Nav2
#Expects the following to live under processed_map_dir
#cuvslam_map cuvgl_map depth edex map_frames occupancy_map.png occupancy_map.yaml

#ARGS:
#processed_map_dir - directory of the processed rosbag2 data
#occupancy_map_name - name of the .yaml file to use with the 2D occupancy map 
#base_map_dir - path where processed rosbag2 data lives

processed_map_dir=$1
occupancy_map_name=$2
base_map_dir="/mnt/nova_ssd/recordings/maps"
vslam_dir=$base_map_dir/$processed_map_dir/cuvslam_map/
vgl_dir=$base_map_dir/$processed_map_dir/cuvgl_map/ 
#vgl_dir=$base_map_dir/cuvgl_map/ 
occupancy_map_path=$base_map_dir/$processed_map_dir/$occupancy_map_name

DOCKER_ARGS+=("-e ROS_DOMAIN_ID=10")
DOCKER_ARGS+=("-e USER")
DOCKER_ARGS+=("-e ISAAC_ROS_WS=/workspaces/isaac_ros-dev")
DOCKER_ARGS+=("-e HOST_USER_UID=`id -u`")
DOCKER_ARGS+=("-e HOST_USER_GID=`id -g`")
DOCKER_ARGS+=("-e LD_LIBRARY_PATH='/workspaces/isaac_ros-dev/ros_ws/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/plugins/aarch64:$LD_LIBRARY_PATH'")

docker run --name perceptor_nav2 --rm --privileged --network host \
    -v /dev/*:/dev/* \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/nova:/etc/nova \
    -v /mnt/nova_ssd/recordings:/mnt/nova_ssd/recordings \
    -v /mnt/nova_ssd/recordings/maps:/mnt/nova_ssd/maps \
    nvcr.io/nvidia/isaac/nova_carter_bringup:fixed_engines \
    ros2 launch nova_carter_bringup perceptor.launch.py use_foxglove_whitelist:=false stereo_camera_configuration:=front_left_right_configuration disable_vgl:=False vslam_load_map_folder_path:=$vslam_dir vgl_map_dir:=$vgl_dir occupancy_map_yaml_file:=$occupancy_map_path vslam_enable_slam:=True 

