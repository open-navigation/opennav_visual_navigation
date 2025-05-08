TODO
	- [ ] Videos
	- [ ] Demos
	- [ ] Software configs/launch/etc
	- [ ] Tutorial

Include Nvidia tutorials, configuration pages, and docs in this document for how to learn more / get more inofmration.


# Lidar-Free, Vision-Based Navigation

In this tutorial, you'll learn how to harness the power of NVIDIA Jetson platforms and `Isaac ROS <https://developer.nvidia.com/isaac/ros>`_ technologies to implement Vision-based Navigation entirely without the use of Lidars, active depth sensors, or other range-providing modalities. 
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
This is made possible by fully leveraging the power of NVIDIA's GPU and the Isaac ROS SDK.

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

Additionally, a utility that Isaac ROS SDK provides, Visual Global Localization, is used to identify the initial starting pose in a localization run when one is not already previously known.
Using the features in at the initial pose, it will match those with the pre-built map to identify the starting pose one-time on startup before continuing the localization session with cuVSLAM - solving the kidnapped robot problem.
This will be used to set the initial pose of the robot before starting navigation sessions.

TODO Chart showing the workflow for this to explain in conclusion

## 0. Nvidia Jetson Setup

### Jetpack

If you don't already have the latest jetpack installed follow the instructions below.
The current version at the time of writing is Jetpack 6.2

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

TODO
  - OR fork this package? https://github.com/open-navigation/opennav_visual_navigation
	- Adjust Nav2 params file
	- Configure the Isaac nodes
	- Launch file to include nodes
	- Docker ... other stuff

TODO
Recommends: 2 cameras (front and back) for 
            2 cameras (sides) for 

TODO people semgnetatino is not just people, any masked segmentation will work (people just current example)

## 2. Configuration



## 3. Initial Environment Mapping

Using launch file ... map ... teleop ... offline process from NV. May take awhile.


## 4. Navigation Testing

Once the initial offline map is complete, we are ready to start navigating!

NVBlox, visual global localizer, localization


## 5. Demonstration

Route? Random point selection? The sample will perform a security patrol looping the space indefinitely with feasible planning and model predictive control until the battery is low, then will autonomously dock to its charging station until sufficiently re-energized to continue the mission.
