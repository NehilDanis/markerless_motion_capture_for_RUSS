# Markerless Motion Capture for Reactive Robotic US

This work aims to achieve the robotic ultrasound while compensating expected and unexpected movements of the object without using any markers. This way, the 3D compounding of the artery of interest will be generated and later can be used to diagnose peripheral vascular disease. The project has two main parts; first, the preoperative CT data and the point cloud captured by the depth sensor will be registered. Then using a segmentation network binary mask of the limb will be generated in real-time; later on, these masks will be compared between each frame using the dice score, and if the dice score drops below the preset threshold, then it is considered that the motion occurred. A signal will be sent to the robot to stop the scan, and later using the ICP algorithm, and the scan trajectory will be updated.

## Hardware used in this project
* a  redundant  robotic arm  (LBR  iiwa  14  R820,  KUKA  GmbH,  Germany)  
* a  US machine (Cephasonics, USA)
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
5) Run `$catkin_make` in the root of your catkin_workspace. You will probably have an issue with PCL. I have used PCL 1.11.1, I have provided a fix for one issue in this version, since I still haven't fix the clang tidy, it is not in master, for now you can pull from [this](https://github.com/NehilDanis/pcl/tree/bug_fix_segfault_executing_multiscale_feature_persistence "PCL") Also in case you need help, you can follow my blog [post](https://nehildanis.github.io/Using-PCL-in-Ubuntu-18.html "pcl in ubuntu18.04")
6) You will also need [Yaml-cpp](https://github.com/jbeder/yaml-cpp/tree/yaml-cpp-0.6.0 "yaml-cpp-0.6.0").
7) If you see any more packages, that is required for the build and you don't have it, please keep installing those, otherwise let's go to the next step.
8) So now, add the ROS_Vision_Controller directory from this project into the src of your catkin workspace.
9) Other packages required to be in the src of your catkin workspace; [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack "iiwa_stack_github"), [easy_handeye](https://github.com/IFL-CAMP/easy_handeye "easy_handeye_github"), [easy_aruco](https://github.com/marcoesposito1988/easy_aruco/pull/3 "easy_aruco") well easy aruco, there is yet another pull request of mine, which I didn't get to chance to finish but pull from my branch, so that you can visualize the pose, position and the marker on the image.
10) After this do `$catkin_make` in your catkin workspace root, and if there are no issues continue.
11) You will need to environments to run python codes, well since when I started this project, I started with Melodic, and I was not able to use Python3, I used [pyro5](https://pyro5.readthedocs.io/en/latest/ "pyro5") to connect my Python2 ROS workspace with the python3 environment that I used for training. Due to this you will need two python environments.<br />
For the python2 environment in your ROS workspace:<br />
`$conda create --name ros_pyro_env python=2.7`<br />
`$conda activate ros_pyro_env`<br />
`$pip install -r src/shape_registration/src/ros_python_requirements.txt`<br />
For the python 3 environment:<br />
`$conda env create -f environment.yaml`<br />
`$conda activate arm_tracking_env`<br />
`$pip install -r tracking_arm/requirements.txt`<br />
12) [Optional] After you created the environments, you can create aliases. Add below lines into your bashrc:
`alias ros_pyro_env="source activate ros_pyro_env"`<br />
`alias arm_tracking="source activate arm_tracking_env"`<br />
13) Good luck :)

## How to use this project

| Commands | Description |
| :----: | :----: |
| `$roslaunch shape_registration registration.launch` | To run the surface registration, please change the registration.launch file according to the path or your CT and artery data |
| First let's start the segmentation network, go to terminal and run the following:<br>`$arm_tracking`<br>start the pyro server:<br>`$pyro5-ns`<br>`$python tracking_arm/subscriber_pyro.py`<br>Now start the ROS part:<br>`$roslaunch shape_registration movement_monitoring_and_tracking.launch`<br>Then start the pyro server in the ros side<br>`$ros_pyro_env`<br>`$rosrun shape_registration publisher_pyro.py`| Movement monitoring section is a bit tricky, follow the commands!|
| `$roslaunch shape_registration image_saving_for_training.launch` | To capture RGB images using the azure kinect, in case you want to create a data set. |
| `rosrun shape_registration find_trajectory_node` | To create scan trajectory |
