#!/usr/bin/env bash
docker run -it --privileged --network host \
    -v /dev/*:/dev/* \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/nova:/etc/nova \
    nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64 \
    /bin/bash
#Don't mount /workspaces because of version conflicts between Jetpack and container
#    nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64 \
#    -v /mnt/nova_ssd/workspaces:/workspaces \
