TODO
	- [ ] Videos
	- [ ] Demos
	- [ ] Software configs/launch/etc
	- [ ] Tutorial
	- [ ] README tutorial link + video + image

TODO demo
  - [ ] Route graph generation / storage in repo
  - [ ] Map generation / storage in repo
  - [ ] Backport Route to Humble
  - [ ] teleop might be automatically launched with the nova recorder for some reason

# Lidar-Free, Vision-Based Navigation

In this tutorial, you'll learn how to harness the power of NVIDIA Jetson platforms, `Isaac ROS <https://developer.nvidia.com/isaac/ros>`_, and `Isaac Perceptor <https://developer.nvidia.com/isaac/perceptor>`_ technologies to implement Vision-based Navigation entirely without the use of Lidars, active depth sensors, or other range-providing modalities. 
Instead, we'll rely solely on passive stereo cameras as the extrinsic sensing source to achieve collision avoidance, localization, and mapping — a powerful and cost-effective alternative. 

This tutorial will guide you through the fundamental concepts behind vision-based navigation, explain how to configure and launch a vision-enabled Nav2 stack optimized for NVIDIA hardware, and culminate in a hands-on demonstration of a mobile robot performing autonomous security patrols.

Whether you're developing for resource-constrained embedded systems or exploring alternatives to expensive sensor suites, this walk-through provides a complete and practical introduction to deploying advanced autonomous navigation with only vision-based perception.

.. note:

  While vision-based solutions offer powerful and efficient capabilities they can face challenges in certain environments. Scenarios involving glass surfaces, featureless corridors, thin obstacles, sudden movements, vibrations, or prolonged stillness under changing lighting conditions may impact performance.
  It's important to thoughtfully evaluate whether a visual navigation approach aligns with the specific demands and conditions of your application to ensure the best results.


## Preliminaries

This tutorial assumes that you have an NVIDIA Jetson platform - such as the Orin NX or AGX Orin - and stereo camera sensor(s).
This tutorial will make use of the `Jetson AGX Orin <https://amzn.to/4k8jiQh>`_ powering the `Segway Nova Carter <https://robotics.segway.com/nova-carter/>`_ robot built in collaboration with NVIDIA for vision-based navigation tasks.
However, another Jetson product may suffice depending on the GPU compute demands placed on it by the number of cameras, resolutions, and models being run. 

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
This is made possible by fully leveraging the power of NVIDIA's GPU and the Isaac ROS & Perceptor SDKs.

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

TODO [include this graphic](https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox_nodegraph.png/) 

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

First, we need to set up a `Isaac ROS Dev<https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html>`_ environment using Docker, as highly recommended by NVIDIA.

## Tooling Setup

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

## Demonstration Setup

We're now ready to launch the container, we can do so via:

.. code: bash

  cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh

TODO: Make sure the rosbag for mapping AND the output occ grid map directories are mounted to the docker container. You can add them in ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/.isaac_ros_dev-dockerargs then restart the container.

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

If you'd like to make modifications for your platform, fork the ``opennav_visual_navigation`` repository, clone your fork instead.

TODO other setup things like NGC resource or package installs

## 2. Software Walkthrough

TODO this is all required to work with a Nova Carter / Nova reference platform. If not, some effort is require to get it to work. The NV tooling is so specific to that you'll have a really hard time getting any of this to work otherwise (recording, launch wrappers, containers, synchronization, etc). I dont love this, but I don't know what else to do. Reformat tutorial as tech demo but not reproducable. SHould I tell NV (launch utils, nesting hardware/nav configs into it, nova tie in) its not really usable?

This project is contains the package ``opennav_visual_nav_demo``, which comprises the application demo leveraging Visual Navigation to perform a security patroling task.
The demonstration leverages the Nova Carter robot, so the hardware is brought up using the ``nova_carter_bringup`` launch file ``navigation.launch.py`` and ``teleop.launch.py`` which launches the robot hardware and other nodes needed for the demonstration.

To adapt to another platform, make a new `my_robot_nav` package which:
* Launches the robot hardware drivers for accepting commands, bringing up sensors, providing transformation tree, etc
* Launches Isaac Perceptor, usually via `isaac_ros_perceptor_bringup` package's `perceptor_general.launch.py` or `perceptor_rgbd.launch.py`
* Launches Nav2 with the appropriate configurations (i.e. removed AMCL for cuVSLAM, Costmap configurations for NvBlox)

Use these launch files to replace `nova_carter_bringup/launch/navigation.launch.py` used in this package.

TODO tutorial specific to nova (+nova carter?) reference platforms?
TODO humble compatible for Jetson, but uses humble-main version for updated featureset

TODO
	- Adjust Nav2 params file
	- Configure the Isaac nodes
	- Launch file to include nodes

  - TODO GIVE UP AND USE JUST NVIDIA WORK THAT DOESN'T GENERALIZE --> THE 'DEMO' IS THE APPLICATION AND SHOWING WORKING, NOT THE SETUP ITSELF?

TODO
Recommends: 3 cameras launched (front, left, right)
            2 cameras for VSLAM
            2 cameras (sides) for 

TODO people semgnetatino is not just people, any masked segmentation will work (people just current example)

YOu've made this really hard to work with and integrate using all these NV specific launch tools. I can't tell what's what and how things are parsed and stored.

## 3. Initial Environment Mapping

TODO Using launch file ... map ... teleop ... offline process from NV. May take awhile.
  online possible? docs seem to imply it is

## 4. Navigation Testing

TODO Once the initial offline map is complete, we are ready to start navigating!

NVBlox, visual global localizer, localization


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

TODO show raoute graph / map of the space

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

TODO Link to the BT XML + image of it

### Configuration

TODO
MPPI, navFN to get onto graph if needed due to diff drive holonomic base (would recommend Hybrid-A* for larger things with non-circular footprint), Goal Checker, NvBlox, 
The control will use a model predictive controller to avoid obstacles and follow the path.
Link to the configuration file

### Videos

TODO video of the demo (roobt view)

TODO video of he demo (rviz view with graph + nvblox + camera?)


## 6. Conclusions & Extensions

In this tutorial, we showed how Nav2 can be used without lidar or depth cameras to conduct vision-only navigation leveraging NVIDA's technologies (Jetson, Isaac ROS, Isaac Perceptor).
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
