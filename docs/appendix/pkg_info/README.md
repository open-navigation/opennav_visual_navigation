### Debug logs, command output and notes for debugging the ISAAC Perceptor model issue

In short, the error is :

```
[component_container_mt-9] [MESSAGE] Chosen primary camera ids: 0 2 4
[component_container_mt-9] [MESSAGE] Selected cameras ids for SLAM: 0 2 4
[component_container_mt-9] IRuntime::deserializeCudaEngine: Error Code 6: API Usage Error (The engine plan file is not compatible with this version of TensorRT, expecting library version 10.7.0.23 got 
[component_container_mt-9] ..)
```

This occurs when running the NVidia ISAAC Perceptor demo following the instructions here:
https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/tutorials_on_carter/demo_perceptor.html

Investigating this shows that the nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64 Docker image has
TensorRT 10.7.0.23 and nvinfer 10.7.0.23+cuda12.6 while the Carter robot native install has 10.3.0.30-1+cuda12.5

In short, the robot is running the official Jetpack 6.2.1 image and the Docker image is an updated development container. 

Both the container and Carter robot are running Jetpack 6.2.1 from pool/main/n/nvidia-jetpack/nvidia-jetpack_6.2.1+b38_arm64.deb 

Approaches tried to date: 

- Removing *.plan files and forcing Perceptor to regenerate them. They are recreated in the old format. 

- Force-upgrading TensorRT and nvinfer on the robot host to 10.7.0.23 . This breaks/removes underlying NOVA/Jetpack config files. 

- Pulling down a fresh copy of the `isaac_ros_dev`  workspace from the ISAAC Github after backing up and archiving the original one the robot was delivered with. Still needs to be tested.   

  

   
