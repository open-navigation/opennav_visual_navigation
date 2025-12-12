#!/usr/bin/env bash
sensor_config=""
if [ -z "$1" ] 
  then
    #sensor_config="nova-carter_hawk-4_imu"
    sensor_config="nova-carter_hawk-4_no-3d-lidar"
else
    sensor_config=$1
fi
echo "Running Nova Carter Recorder in headless mode using sensor_config=$sensor_config"
docker run --privileged --network host \
    -v /dev/*:/dev/* \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/nova:/etc/nova \
    -v /mnt/nova_ssd/recordings:/mnt/nova_ssd/recordings \
    nvcr.io/nvidia/isaac/nova_carter_bringup:fixed_engines \
    ros2 launch isaac_ros_nova_recorder nova_recorder.launch.py \
    config:=$sensor_config \
    headless:=True
#nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64 \
#-v /mnt/nova_ssd/workspaces:/workspaces \
#^^^Might be needed for isaac_ros_assets, not sure why we're even mounting isaac_ros_assets
#on the container since it doesn't seem to have anything to do with 
#What's going on the current image 
