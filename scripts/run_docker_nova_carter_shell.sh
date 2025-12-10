#Launch an interactive Docker container 
#Using an image w/ version-appropriate ESS .engine and .plan files 
#!/usr/bin/env bash
docker run -it --privileged --network host \
    -v /dev/*:/dev/* \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/nova:/etc/nova \
    -v /mnt/nova_ssd/recordings:/mnt/nova_ssd/recordings \
     nvcr.io/nvidia/isaac/nova_carter_bringup:fixed_engines
    /bin/bash
