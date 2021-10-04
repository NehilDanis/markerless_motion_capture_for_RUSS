# Markerless Motion Capture for Reactive Robotic US

This work aims to achieve the robotic ultrasound while compensating expected and unexpected movements of the object without using any markers. This way, the 3D compounding of the artery of interest will be generated and later can be used to diagnose peripheral vascular disease. The project has two main parts; first, the preoperative CT data and the point cloud captured by the depth sensor will be registered. Then using a segmentation network binary mask of the limb will be generated in real-time; later on, these masks will be compared between each frame using the dice score, and if the dice score drops below the preset threshold, then it is considered that the motion occurred. A signal will be sent to the robot to stop the scan, and later using the ICP algorithm, and the scan trajectory will be updated.

## Hardware used in this project
* a  redundant  robotic arm  (LBR  iiwa  14  R820,  KUKA  GmbH,  Germany)  
* a  USmachine (Cephasonics, USA)
* an RGB-D camera (AzureKinect, Microsoft Corporation, USA).


## How to install this project
**PC setup**
- Ubuntu 18.04 
- ROS Melodic
- CMake version 3.10.2

1) If you do not have ROS installed in your system, follow [this](http://wiki.ros.org/ROS/Installation "ROS Installation"). In this project ROS1, Melodic distribution is used.
2) Once you are done, create a catkin workspace, for that you can follow the [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace "catkin_ws Generation") from the official ROS website. 
3) Since Azure Kinect is used in this project, you need to also get the Azure Kinect ROS driver; this is the official [github page](https://github.com/microsoft/Azure_Kinect_ROS_Driver "Azure Kinect ROS Driver"). Please select the branch according to your ROS distribution. Follow the instructions. In case you face with some issues, I suggest you to follow these [notes](https://gist.github.com/madelinegannon/c212dbf24fc42c1f36776342754d81bc "Azure Kinect ROS driver fix"), they were  helpful to me. 
4) Add the shape_registration and directory from this project into the src of your catkin workspace.
5) Run `catkin_make` in the root of your catkin_workspace. You will probably have an issue with PCL. I have used PCL 1.11.1, I have provided a fix for one issue in this version, since I still haven't fix the clang tidy, it is not in master, for now you can pull from [this](https://github.com/NehilDanis/pcl/tree/bug_fix_segfault_executing_multiscale_feature_persistence "PCL") Also in case you need help, you can follow my blog [post](https://nehildanis.github.io/Using-PCL-in-Ubuntu-18.html "pcl in ubuntu18.04")
6) You will also need [Yaml-cpp](https://github.com/jbeder/yaml-cpp "yaml_cpp_github").
7) If you see any more packages, that is required for the build and you don't have it, please keep installing those, otherwise let's go to the next step.
8) So now, add the ROS_Vision_Controller directory from this project into the src of your catkin workspace.
9) Other packages required to be in the src of your catkin workspace; [iiwa_stack](https://gitlab.lrz.de/CAMP_ROS/iiwa_stack "iiwa_stack_gitlab"), [easy_handeye](https://github.com/IFL-CAMP/easy_handeye "easy_handeye_github"), [easy_aruco](https://github.com/marcoesposito1988/easy_aruco/pull/3 "easy_aruco") well easy aruco, there is yet another pull request of mine, which I didn't get to chance to finish but pull from my branch, so that you can visualize the pose, position and the marker on the image.
10) Good Luck :D

## How to use this project
