#Bare bring up of Perceptor w/ out mapping or ROS2 Nav2
#JIC you want to use Perceptor w/ NVIDIA visual_slam
#!/usr/bin/env bash
docker run --privileged --network host \
    -v /dev/*:/dev/* \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/nova:/etc/nova \
    -v /mnt/nova_ssd/recordings:/mnt/nova_ssd/recordings \
    nvcr.io/nvidia/isaac/nova_carter_bringup:fixed_engines \
    ros2 launch nova_carter_bringup perceptor.launch.py use_foxglove_whitelist:=false \
