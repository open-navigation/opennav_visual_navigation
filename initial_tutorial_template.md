TODO
	- [ ] Videos
	- [ ] Demos
	- [ ] Software configs/launch/etc
	- [ ] Tutorial



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

  * 

		- depth data use-cases (mapping/reconstruction)
		- mapping (generate map),
		- localization (localize within a map), 
		- 3D Scene Reconstruction (cost grid)

	- What Nvidia replaces to make this possible (table?)
		- Depth Perception --> Lidar depth points and/or depth camera. 1:1 replacement.
		- VSLAM / Localization / Visual Global Localization --> AMCL, SLAM Toolbox. 1:1 replacement.
		- NVBlox --> Voxel Layer. Does the reconstruction and sent to costmap2d just as the method for combining it with potentially other data and as the data structure planner/controllers expect to standardized represnetaiton.

  - Chart showing the workflow for this to explain

Visual Global Localization: visual features not lidar to do intiial pose finding. Just run for 1-shot here's where you are in the map

cuSLAM does VIO and SLAM mode. cuVSLAM does global localization based on output of cuVGL. Pure localization: no updating features or evolving the map

## 0. Nvidia Jetson Setup

### Jetpack

If you don't already have the latest jetpack installed follow the instructions in the collapsed section below (Jetpack 6.2 at the time of writing).

>>>> COLLAPSED

On Jetpack 6.0+? https://docs.nvidia.com/jetson/jetpack/install-setup/index.html#upgrade-jetpack apt install new version easy enough.

Else, Install the SDK Manager at https://developer.nvidia.com/sdk-manager either as a docker image or debian on your developer machine.

Create or log into your nvidia account.

Plug in your Jetson via USB-C from the USB-C port in the back of the Jetson (it is used for bootloading)

Follow the instructions on this page, paying attention to use the right Jetson product: https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html. 

Pay attention that the IP address they're asking for is the wired address with the USB-C connection, not its wireless IP.

Note: if you have any problems consider flashing script (TODO)

>>>>

### Power Modes

If not already setup in Max Power Mode, we recommend you do so now to be able to leverage the full power of the Jetson. If in a highly power constrained application, consider which power setting make most sense for your power resources, compute requirements, and application duration. Revisit this after the demonstration to optimize for your requirements and power needs.

### Cameras

We love stereolabs ... but others work (include? How to do this without pissing off Stereolabs. Ping them for a tutorial usign their spatial AI?)

VSLAM requires tightly time synchronized setup. Recommend HAWK, ZED, Orbecc Rig. Resolution and calibration sensitive.
  Also needs multiple cameras to work well from their experience 

NVBLOX can work well with 1 camera. Less sensitive. Needs pose esetimate that VSLAM provides, but can be from any source. But trusts it word of god without correction.

 compatible with Argus or similar capture and synchronization technol

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
