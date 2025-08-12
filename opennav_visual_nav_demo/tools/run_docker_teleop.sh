#!/usr/bin/env bash
docker run --privileged --network host \
    -v /dev/*:/dev/* \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/nova:/etc/nova \
    nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64 \
    ros2 launch nova_carter_bringup teleop.launch.py
