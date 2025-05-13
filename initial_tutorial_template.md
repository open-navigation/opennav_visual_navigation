TODO
	- [ ] Videos
	- [ ] Demos
	- [ ] Software configs/launch/etc
	- [ ] Tutorial
	- [ ] README tutorial link + video + image

TODO demo
  - [ ] Route graph, maps in repo
  - [ ] Backport Route to Humble

NV feedback
  - Too tied into NV tooling
    - launch uses an NV wrapper that is not clear how to decypher. Also hard to integrate with as a result for anyone else.
  - Software is too single-purpose specific
    - None works without Nova
    - Only works if exactly 1 workflow is followed setting env var or configs for applications (fragile)
    - Things are nested in uninutitive ways that make it impossible for reuse. Recorder launches teleop of robot base. perceptor includes Nav2 + configs can't rip out. Hardware is launched when enabling perception stack (backwards) that make it inoperable for anyone else - can't do anything without restarting.
    - Can't really be reapplied (at least not easily) to any other stiuation but the one NV created to make the demo
  - Docs are twisted, interconnected and non-linear, it took me hours of back and forth to extract information and still feel like its not telling me the full story on configurations, setup, how to work with it outside of the 1 situation using the nova platform, etc
  
  --> Had to change from a codebase intended as a forkable platform that folks could modify for their situation as a working demo to build off of and INSTEAD make it a technology demonstration of only the 1 situation we have with the Nova Carter to show its possible, but only in this narrow situation. Tech demo to show its possible rather than a usable platform for users to build fromf for their unique needs.

# Lidar-Free, Vision-Based Navigation

In this tutorial, you'll see how to use the NVIDIA Jetson, `Isaac ROS <https://developer.nvidia.com/isaac/ros>`_, `Isaac Perceptor <https://developer.nvidia.com/isaac/perceptor>`_, and `NVIDIA Nova <https://nvidia-isaac-ros.github.io/nova/index.html>`_ technologies to implement Vision-based Navigation entirely without the use of Lidars, active depth sensors, or other range-providing modalities. 
Instead, we'll rely solely on passive stereo cameras as the extrinsic sensing source to achieve collision avoidance, localization, and mapping — a powerful and cost-effective alternative. 

This tutorial will guide you through the fundamental concepts behind vision-based navigation, explain how to configure and launch a vision-enabled Nav2 stack, and culminate in a hardware demonstration of a mobile robot performing autonomous security patrols.

Whether you're developing for resource-constrained embedded systems or exploring alternatives to expensive sensor suites, this walk-through provides a complete introduction to deploying advanced autonomous navigation with only vision-based perception using a NVIDIA Nova reference platform, including stereo cameras and Jetson AGX Orin.

.. note:

  While vision-based solutions offer powerful and efficient capabilities they can face challenges in certain environments. Scenarios involving glass surfaces, featureless corridors, thin obstacles, sudden movements, vibrations, or prolonged stillness under changing lighting conditions may impact performance.
  It's important to thoughtfully evaluate whether a visual navigation approach aligns with the specific demands and conditions of your application to ensure the best results.


## Preliminaries

This tutorial assumes that you have an NVIDIA Jetson - such as the Orin NX or AGX Orin - and stereo camera sensor(s) compatible with the Nova reference platform.
This tutorial will make use of the `Jetson AGX Orin <https://amzn.to/4k8jiQh>`_ powering the `Segway Nova Carter <https://robotics.segway.com/nova-carter/>`_ robot built in collaboration with NVIDIA for vision-based navigation tasks.
However, another Jetson product may suffice depending on the GPU compute demands placed on it by the number of cameras, resolutions, and models being run.
Applying these technologies to a non-Nova design is possible using the general concepts and designs in this tutorial, however it involves a great deal of unique development as the launch files and nodes provided by NVIDIA assume this.

### Concepts

Navigation is performed by combining sensor data map(s) of the environment, and task intent to successfully move a robot to a final destination.
Major elements of this include Path Planning, Control, Perception, Localization, and the Autonomy system to pull them all together.
If you want to learn more about Navigation, checkout :ref:`concepts`.

The elements that make use of sensor data are Perception and Localization.
Planning and Control make use of derivative information from these subsystems, such as the robot's current pose or information about the location of potential obstacles, but not the direct data itself.
Structures like the environmental model fuses sensor data from many sensors together into a representation that is data agnostic, which can be used for planning and control algorithms to make decisions based off of.
In Nav2, that is commonly the Costmap package, but could be height maps, voxel grids, and so on.

Thus, for Visual Navigation, there are two major areas which need adjustment from Lidar and Depth-based navigation into Vision-based navigation: Perception and Localization.
We must replace the existing solutions for the following with Vision-based systems:

  * **Data Acquisition**: Compute disparity information and derived results from stereo camera information
  * **Data Fusion**: Combine multiple camera feeds into an environmental model for use in planning and control
  * **Mapping**: Creating a globally-consistent view of the environment using camera data
  * **Localization**: Using a globally-consistent view of the environment to later localize the robot within during task execution

Currently, these are solved in Lidar-based solutions using Costmap2D, a SLAM library like `SLAM Toolbox <https://github.com/SteveMacenski/slam_toolbox>`_ or Cartographer, and a global localizer like AMCL.

TODO diagram showing these concepts visually - with current and replacement solutions present?


### Nvidia Technologies

NVIDIA provides the necessary technologies to replace the existing Lidar-based navigation solutions.
This is made possible by leveraging the power of NVIDIA's GPU and the Isaac ROS & Perceptor SDKs.

TODO Table for 1:1 Nvidia replaces to make this possible.

**Data Acquisition**:

Depth acquisition with Lidar- and Depth-based sensors is relatively straight forward.
For the majority of cases, a ROS driver will connect to the sensor and provide the depth data in a standard format like a Depth Image or PointCloud2.
Sometimes post-processing is needed to remove noise or other artifacts to improve later 3D reconstruction.

In the Vision-based navigation, we instead require more stages to obtain both the sensor data and derived results needed for 3D reconstructions to build the environmental model required for global path planning and local trajectory planning (control). 

Data is first acquired via the Jetson's libargus or a sensor manufacturer provided library to obtain sensor data in a low-latency, time synchronized way to enable accurate information for Visual mapping and localization purposes.
This is key for good performance of a vision-based solution and many sensors are supported.
The disperity is then estimated using Isaac's ``isaac_ros_ess``, which computes a GPU accelerated, deep-learned stereo disparity image.
Finally, ``isaac_ros_stereo_image_proc`` converts the disparity image into a Depth Image used for later 3D reconstruction.

.. note:

  ```isaac_ros_stereo_image_proc`` may also compute a PointCloud2 as well if an application calls for pointcloud rather than depth image format.

**Data Fusion**:

Once we'd obtained the depth information from the stereo pair, we can use this for environmental model construction and updates so we can leverage knowledge about the environment to make intelligent planning and control choices.
While definitionally 3D Reconstruction methods may not require depth information from camera feeds, most modern and robust solutions require it, hence the need for the Isaac SDK's depth estimation pipeline.

NVIDIA provides a great 3D Reconstruction solution called `NvBlox <https://github.com/nvidia-isaac/nvblox>`_.
NvBlox is a GPU accelerated signed-distance field library which can be used to generate environmental models using voxel grids.
This can take in multiple depth images from stereo camera pairs and populate a 3D environmental representation.

It can also accept an optional semantic segmentation mask to detect people, robots, or other dynamic objects in the scene to remove them from the environmental model's update.
These dynamic obstacles are then later re-inserted at the end of the update to avoid artifacts in environmental updates related to dynamic obstacles without the need of expensive clearing logic.
Common demonstrations show this with a particular human segmentation model, but any model may be used trained to segment out any number of object classes.

TODO
.. image:
  file: graphics/isaac_ros_nvblox_nodegraph.png

The NvBlox model is hosted inside of the Costmap2D package as a Costmap Layer Plugin used to update the occupancy grid model for use in planning and control.
Future updated to Nav2 may make it possible to use 3D environmental models natively rather than reducing dimensionality for use-cases that require 3D collision checking like mobile manipulation and traversibility estimation.

This removes the need to work with a Voxel Layer, Spatio-Temporal Voxel Layer, Obstacle Layer, or other sensor processing layers to mark and clear data from the occupancy grid.

**Mapping**:

Mapping is crutial for long-term planning to understand the environment and know how to navigate from a given point to any other point in a space most optimally.
While short-term navigation tasks with immediate visibility of the space may not require a pre-built map, most practically deployed applications require either (1) time-optimal execution and cannot get lost attempting to navigate down incorrect areas that will not lead to a solution or (2) operate in large spaces where the target poses are not commonly visible from current sensor data to establish a route to the goal.

`cuVSLAM <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html>`_ is used for visual SLAM (VSLAM) to create and save a map of the environment from sensor data.
This is a pure visual mapping solution that uses only stereo camera images and IMU data to map the environment using stereo feature matching.
Due to the computational demands of the mapping process on moderately large spaces, this is completed offline from a teleoperated data collection run.
This fully replaces 2D, 3D, or other types of Lidar-SLAM.

**Localization**:

cuVSLAM is also used for run-time pure localization within the feature map generated during the mapping run.
This can run in excess of 30 fps with 4 stereo camera pairs, or even faster with fewer on the Jetson AGX Orin.

Additionally, a utility that Isaac ROS SDK provides, Visual Global Localization (cuVGL), is used to identify the initial starting pose in a localization run when one is not already previously known.
Using the features in at the initial pose, it will match those with the pre-built map to identify the starting pose one-time on startup before continuing the localization session with cuVSLAM - solving the kidnapped robot problem.
This will be used to set the initial pose of the robot before starting navigation sessions.
It may also be used to relocalize the robot during runtime as well, which can be run in just under 1 second.

TODO Chart showing the workflow for this to explain in conclusion

## 0. Nvidia Jetson Setup

### Jetpack

If you don't already have the latest jetpack installed follow the instructions below.
The current version at the time of writing is Jetpack 6.2 with Isaac 3.2.

If the Jetson is currently running Jetpack 6.0 or higher, `please use this guide to upgrade using apt to Jetpack 6.2 <https://docs.nvidia.com/jetson/jetpack/install-setup/index.html#upgrade-jetpack>`_.
Otherwise, the `NVIDIA SDK Manager <https://developer.nvidia.com/sdk-manager>`_ is required to upgrade, either as a docker image or debian to run on a developer computer.

.. note:

  Be sure to have the Jetson's USB-C port used for bootloading accessible if upgrading using the SDK Manager. Use the IP addresses of this wired connection, not over a local WAN.

### Nova Init

If not already setup, make sure to install and configure Nova Init using `the following instructions <https://nvidia-isaac-ros.github.io/nova/nova_init/index.html>`_. 

### Power Modes

If not already setup in Max Power Mode, we recommend you do so now to be able to leverage the full power of the Jetson.
If in a highly power constrained application, consider which power setting make most sense for your power resources, compute requirements, and application duration.
Revisit this after the demonstration to optimize for your requirements and power needs.

### Compatible Cameras

There are several cameras that are compatible with this workflow.
We recommend the `Stereolabs ZED <https://www.stereolabs.com/>`_ cameras for their native integration with the NVIDIA ecosystem and ease of use.
Stereolabs also provides a powerful SDK of Vision and Spatial AI features which can be further leveraged to increase the functionality and intelligence of a vision-based mobile robot.
However, stereo cameras from `Leopard Imaging <https://leopardimaging.com/leopard-imaging-hawk-stereo-cameras/>`_ are also compatible and supported.

Cameras with tight integration and synchronization are required to work with cuVSLAM due to its timing constraints to achieve accurate results.
Thus, we recommend using one of these compatible options.
However, if you'd like to use NvBlox without cuVSLAM and using another localization and mapping solution, a broader range of camera options are available such as the Realsense.
NvBlox uses the pose estimates that cuVSLAM provides, but those pose estimates can be from any source.
It uses these pose estimates to place the sensor data in the scene to populate the environmental model.
NvBlox can work well on just a single stereo camera, but cuVSLAM typically requires two or more cameras to see enough of the scene to obtain robust results.

## 1. Initial Setup

First, we need to set up a `Isaac ROS Dev <https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html>`_ environment using Docker, as highly recommended by NVIDIA.

### Tooling Setup

.. code: bash

  sudo systemctl daemon-reload && sudo systemctl restart docker
  sudo apt install git-lfs
  git lfs install --skip-repo

  mkdir -p  /mnt/nova_ssd/workspaces/isaac_ros-dev/src
  echo "export ISAAC_ROS_WS=/mnt/nova_ssd/workspaces/isaac_ros-dev/" >> ~/.bashrc
  source ~/.bashrc

This sets up Git LFS (large file storage) for downloading weights and files stored if the Isaac ROS source repositories.
It also creates a developer workspace ``isaac_ros-dev`` either on an externally mounted SSD or on the local computer.
It is recommended to use an externally mounted SSD to have sufficient storage to run this demonstration.
If one is not available, replace ``/mnt/nova_ssd/workspaces/isaac_ros-dev/`` with ``${HOME}/workspaces/isaac_ros-dev/`` in the commands above.
This also sets an environmental variable ``ISAAC_ROS_WS`` which is used to mount the workspace to the Isaac ROS containers and in other Isaac workflows, so it is important to always have that set.

Next, we're going to clone the ``isaac_ros_common`` package which contains key elements of Isaac ROS, including the dockerfiles and scripts needed to run Isaac in the Dev environment.

.. code: bash

  cd ${ISAAC_ROS_WS}/src && git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

If working with a Nova Carter device, `do the following <https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#repositories-setup-optional>`_ as well to setup the docker configs.

### Demonstration Setup

We're now ready to launch the container, we can do so via:

.. code: bash

  cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh

TODO: Make sure the rosbag for mapping AND the output occ grid map directories are mounted to the docker container. You can add them in ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/.isaac_ros_dev-dockerargs then restart the container.
echo -e '-v /mnt/nova_ssd/recordings:/mnt/nova_ssd/recordings' > ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/.isaac_ros_dev-dockerargs
echo -e '-v /mnt/nova_ssd/maps:/mnt/nova_ssd/maps' > ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/.isaac_ros_dev-dockerargs

Once we've obtained and setup Isaac, we can add in the ``opennav_visual_navigation`` project as a starting point.

.. code: bash
  # Clones the project and creates a colcon_ws relative to your pat 
  cd ${ISAAC_ROS_WS}/src && git clone git@github.com:open-navigation/opennav_visual_navigation.git
  
  # Obtains dependencies
  rosdep init  # If not previously initialized
  rosdep update
  rosdep install -r -y --from-paths colcon_ws/src --ignore-src

  # Initially builds the workspace
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install

TODO other setup things like NGC resource or manual package install steps

## 2. Software & Workflow Walkthrough

This project is contains the package ``opennav_visual_nav_demo``, which comprises the application demo leveraging Visual Navigation to perform a security patroling task.
The demonstration leverages the Nova Carter robot, so the hardware is brought up using the ` ``nova_carter_bringup`` <https://github.com/NVIDIA-ISAAC-ROS/nova_carter/tree/main/nova_carter_bringup>`_ launch file ``navigation.launch.py`` and ``teleop.launch.py`` which launches the robot hardware and other nodes needed for the demonstration.
This has been preconfigured with Nav2, Isaac Perceptor, and is highly integrated with the Nova reference platform.
Please reference this package for more information.

To adapt to another platform, make a new `my_robot_nav` package which:
* Launches the robot hardware drivers for accepting commands, bringing up sensors, providing transformation tree, etc
* Launches Isaac Perceptor, usually via `isaac_ros_perceptor_bringup` package's `perceptor_general.launch.py` or `perceptor_rgbd.launch.py`
* Launches Nav2 with the appropriate configurations (i.e. removed AMCL for cuVSLAM, Costmap configurations for NvBlox)

Use these launch files to replace `nova_carter_bringup/launch/navigation.launch.py` used in this package.

The launch file ``teleop_mapping_launch.py`` is used to teleoperate the robot using a joystick to collect data for a mapping session.
Once a dataset is created, ``tools/create_map.sh`` will be used to create the SLAM map offline.
``visual_nav_demo_launch.py`` will be used for the live localization and navigation demostration using the visual navigation capabilities.
The demonstration application (``patrol_application.py``) and its configurations will be discussed in a later section.

Outside of the ROS package you will find the data from this run including the VSLAM map, occupancy grid map, and route graph files used in the demonstration as a reference.

The demonstration is setup to utilize the front, left, and right stereo cameras for localization with VSLAM and 3D reconstruction using NvBlox using live use. 
While the rear camera would also provide useful information (and is also used during the VSLAM mapping dataset), it adds additional computation beyond what the AGX Orin can handle in real-time with the present Isaac release.
This may be improved at a future time or when using newer Jetson platforms such as the Thor.
The 2D and 3D lidars are disabled and not used anywhere in this work.

## 3. Initial Environment Mapping

### Data Collection

The first step is to generate a dataset to create the VSLAM map during offline processing.
When working in realistically large spaces, this cannot be done on the Jetson in real-time, so it is necessary to teleoperate the robot for the initial mapping session rather than completed in real-time.

Tips and Tricks:
  * Start datasets with 10 seconds at the starting pose before moving the robot so it may be used in the future for localization testing
  * For each 5x5m area, drive for around 1 minute
  * Drive in closed loops and make sure to capture data at more than one angle (i.e. drive in different directions to obtain different viewpoints)
  * Don't drive for a long time in a straight line, loop back to create loop closures

TODO
.. image:
  file: graphics/mapping1.gif
  file: graphics/mapping2.gif

When ready, inside of the docker image from before, run the following and joystick your robot through the space:

.. code: bash

    ros2 launch opennav_visual_nav_demo teleop_mapping_launch.py

TODO video joysticking

### Data Processing

After the dataset has been collected, it can be post-processed to generate the cuVSLAM map for localization and cuVGL map used for initial global localization. It can be done via ``./tools/create_map.sh <PATH_TO_ROSBAG> <PATH_TO_OUTPUT_FOLDER>``. For example using the recommended directories:

.. code: bash

    ./create_map.sh /mnt/nova_ssd/recordings/my_recorded_data.bag /mnt/nova_ssd/maps

TODO you should see .... output / files. PICTURES of directory/files to explain.

To create the data for cuVGL, we need to perform 2 steps. First, to extract features from the dataset and finally create the global localization map.
For this, we will use ``./tools/create_vgl_map.sh <PATH_TO_SENSOR_ROSBAG> <PATH_TO_POSE_ROSBAG> <PATH_TO_OUTPUT_FOLDER>`` with example below.

.. code: bash

    ./create_vgl_map.sh /mnt/nova_ssd/recordings/my_recorded_data.bag /mnt/nova_ssd/maps

TODO you should see .... output / files. PICTURES of map.
TODO pose topic in script, the directories to use, etc.

.. note:

  This step may take some time and use the full compute power of the Jetson. It is recommended to not have other workloads on the Jetson while this is processing (and may want to take a lunch break).

### Navigation Graph

Now that we have the data analysis completed to do localization and initial global localization, we can now generate the navigation route graph used in this example.
We will annotate the global map file with the navigation graph of routes to perform during a patrol mission.
In this, we'll mark patrol points and connect them with directional or bidirectional edges for how the robot can achieve these patrol points on the graph.

For more details, see :ref:`route_server_tools` for step-by-step instructions for how to create a graph using QGIS or a Web UX.

TODO graph image

## 4. Navigation Testing

Now that this initial setup is complete, we're ready to start navigating usign visual localization and collision avoidance! 

Simply run the main demonstration launch file and see it in action!

.. code: bash
  
    ros2 launch opennav_visual_nav_demo visual_nav_demo_launch.py
    
TODO video of it navigating around for the first time using all of this to go from A to B in freespace

## 5. Demonstration

### Application Description

The demonstration uses the new (as of May 2025) feature: Nav2 Route Server.
This uses a pre-defined graph of nodes and edges to use for long-range routing rather than global planning through a space.
This can be usefulf for spaces that are too large to map, too large to realistically represent with a dense environmental model, applications where determinisim in routing is necessary, or applications where its important to limit where the robot is allowed to navigate (lanes, zones, etc).

The visual navigation demonstration uses the route server to patrol a space using this route graph.
We select a random node for demonstration purposes to allow the robot to navigate across the space frequently, but a structured policy could easily be created to optimize the patrol routing and task completion for specific applicaitons.
The robot will continue to patrol indefinitely until the battery level is low.
Once it is below a threshold, it will return to the dock to charge automatically.
After its above a set threshold, it will continue the patrolling task until canceled by the user application (i.e. joystick button for demonstration purposes).
After which, the robot will also automatically dock with the charger and wait for its next instruction for a full closed-loop behavior.

Because Visual SLAM has difficulty localizing a robot too far from poses taken during the feature map generation phase, this Route Server is a perfect pairing of technologies as long as the mapping session is similarly representative of the later graph traversals.

The demonstration was completed using the same instructions above for the initial mapping phase and larger task execution with vision-based navigation & localization.

### Behavior Tree

This demonstration uses the route graph within the Behavior Tree to navigate between nodes using the navigation graph rather than freespace planning to ensure determinism of execution to keep the robot in approved areas & capturing necessary security data.
The route is computed on the initial request as well as when the route itself is found to be in collision in order to reroute around obstacles.

Within the behavior tree, if the robot is too far away from a given graph node to start, the behavior tree will automatically freespace plan the robot to the nearest node to get on the graph ('first meter').
Similarly, if the goal pose does not conincide with a graph node, the behavior tree will also freespace plan to get to the final pose from the nearest graph node ('last meter') using a kinematically feasible planner.

This behavior tree uses the route as the global path to follow, so it is post-processed using the Nav2 Smoother to smooth out corners and turns to be more easily navigable.
In other applications, the route nodes and edges can be used directly to seed global planning in order to obtain freespace plans using the route as a general guide for the overall behavior, but still respond to cost grid or behavioral constraints.

As with other Nav2 behavior trees, it also has contextual and global recovery behaviors to handle faults.
This tree can be found in ``opennav_visual_nav_demo/behavior_tree/vis_nav_demo_bt.xml`` and is shown below:

.. image:
  filename: graphics/vis_nav_bt.png TODO

### Configuration

This is configured to use the following algorithms:

=================================
Planner:   | Route Server, NavFn
-----------|---------------------
Controller:| MPPI Controller
=================================

The Route Server is used for long-term planning of the route intent to achieve the goal using the navigation graph while NavFn is exclusively used to go to the first node and/or the goal pose should the starting pose or the goal pose be off the navigation graph.
For this demonstration, no nodes or poses will be off of this graph, but it is useful to show a demonstration for how it could be used and as a backup method should it be required in exceptional situations.
Since this is used for short term and in largely open spaces, NavFn was selected over Hybrid-A* to allow the use of the differential drive robot nature to rotate in place.

The controller selected was the MPPI controller due to its superior performance in dynamic environments and with emergent behaviors due the the Model Predictive Control critic functions.
This allows the robot to leave the graph slightly to avoid real-time obstacles, have intelligent behavior to get out of complex situations, and return to the graph when otherwise not attempting to resolve a problematic situation.
The Goal Checker was configured with a goal tolerance of 2 PI since the route graph nodes have no orientation information to meaningfully achieve in the route graph when not using pose-based goal locations.
This allows us to achieve the goal once the robot is within spatial tolerance to the goal, which will be achieved aligned with the last traversed edge in the graph. When the robot continues to navigate then, it will be oriented with the graph.

### Videos

TODO video of the demo (roobt view)

TODO video of he demo (rviz view with graph + nvblox + camera?)


## 6. Conclusions & Extensions

In this tutorial, we showed how Nav2 can be used without lidar or depth cameras to conduct vision-only navigation leveraging NVIDA's technologies (Jetson, Isaac ROS, Isaac Perceptor, Nova reference platform).
To leverage even more vision features during Visual Navigation, you can also use the Isaac SDK, ZED SDK, or other AI technologies to leverage the GPU for:

  * Object detection or semantic segmentation
  * Ground or freespace segmentation
  * Explore other VSLAM, VIO, or 3D Mapping technologies

### Resources

More detailed information can be found in the following documentation:

* `Isaac ROS Visual SLAM <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html>`_
* `Isaac ROS NvBlox <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html>`_
* `Isaac ROS Visual GLobal Localization <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_ros_visual_global_localization/index.html>`_
* `Isaac ROS Stereo Image Proc <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_image_pipeline/isaac_ros_stereo_image_proc/index.html>`_
* `Isaac Perceptor <https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/index.html>`_
* `Isaac ROS Mapping <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_mapping_ros/index.html>`_

Related GitHub repositories can be found here:

* https://github.com/NVIDIA-ISAAC-ROS/nova_carter
* https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mapping_and_localization
* https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/tree/main
* https://github.com/NVIDIA-ISAAC-ROS/isaac_perceptor/tree/main/isaac_ros_perceptor_bringup
* https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nova
* https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox

More demonstrations can be found here:
* `Tutorial: Mapping and Localization with Isaac Perceptor <https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/tutorial_mapping_and_localization.html>`_
* `Tutorial: Camera-based 3D Perception with Isaac Perceptor on the Nova Carter <https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/tutorials_on_carter/demo_perceptor.html>`_
* `Tutorial: Autonomous Navigation with Isaac Perceptor and Nav2 on the Nova Carter <https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/tutorials_on_carter/demo_navigation.html>`_


TODO:
    export RECORDINGS_FOLDER=/mnt/nova_ssd/recordings
    export MAPS_FOLDER=/mnt/nova_ssd/maps
    docker pull nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64
    docker run --privileged --network host \
      -v /dev/*:/dev/* \
      -v /tmp/argus_socket:/tmp/argus_socket \
      -v /etc/nova:/etc/nova \
      -v $RECORDINGS_FOLDER:$RECORDINGS_FOLDER -v $MAPS_FOLDER:$MAPS_FOLDER \
      nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64 \
      ros2 launch nova_carter_bringup perceptor.launch.py stereo_camera_configuration:=front_left_right_configuration disable_vgl:=False vslam_load_map_folder_path:=<PATH_TO_MAP_FOLDER>/cuvslam_map/ vgl_map_dir:=<PATH_TO_MAP_FOLDER>/cuvgl_map/ occupancy_map_yaml_file:=<PATH_TO_MAP_FOLDER>/occupancy_map.yaml vslam_enable_slam:=True