
---

# Udacity's Self-Driving Car Engineer Nanodegree Program 

[![Udacity - Self-Driving CarNanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---
## Team Early Birds: Final Project - Programming a Real Selfdriving Car (Capstone)
### The Team Members:



\# | Member                  | Udacity Account Mail          | Time Zone               | Github Contributions                                                |
---| ---                     |---                            | ---                     | ---                                                                 |
 1 | Andreas Wienzek	     | andreas.wienzek@gmail.com     | UTC+01:00 (Germany)     | https://github.com/AndysDeepAbstractions/Early_Birds_CarND-Capstone |
 2 | Arjaan Buijk            | arjaan.buijk@gmail.com        | UTC-05:00 (Detroit)     | https://github.com/ArjaanBuijk/Early_Birds_CarND-Capstone           |
 3 | Sachit Vithaldas        | macintoshsac@gmail.com        | UTC-07:00 (California)  | https://github.com/sachitv/Early_Birds_CarND-Capstone               |
 4 | Zeeshan Anjum           | zeeshananjumjunaidi@gmail.com | UTC+03:00 (Bahrain)     | https://github.com/zeeshananjumjunaidi/Early_Birds_CarND-Capstone   |
 5 | Nick Mariano            | nmar95@gmail.com              | UTC-06:00 (Dallas)      | https://github.com/nmar95/Early_Birds_CarND-Capstone                |

 
Team Submission Repository: https://github.com/AndysDeepAbstractions/Early_Birds_CarND-Capstone

Slack Communication: https://earlybirds-sdcnd.slack.com/


---
## Overview
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).
   
![alt text](./imgs/carla.jpg?raw=true "Carla")

Udacity Self-Driving Car Hardware Specs:
- CAR Lincoln MKZ
- CPU Intel Core i7-6700K CPU @ 4 GHz x 8
- MEMORY 31.4 GiB 
- GRAPHICS Nvidia TITAN X 
- OS ROS 64-bit 

---
## Simulation
The software was tested with the [System Integration Simulator](https://github.com/udacity/CarND-Capstone/releases "System Integration Simulator") and tested using [ROS Bags](http://wiki.ros.org/Bags "ROS Bags")
![alt text simulation image](./imgs/simulator.png?raw=true "Simulation")

Testing using the System Integration Simulator

![alt text rosbag image](./imgs/ros_bag.png?raw=true "Simulation")

Testing using ROS bags that were recorded at the test site


[![IMAGE ALT TEXT yt](http://img.youtube.com/vi/AeTWVj-u7h0/0.jpg)](http://www.youtube.com/watch?v=AeTWVj-u7h0)

Video showing Simulation Processes

---
## ROS Nodes Description
The following is a system architecture diagram showing the ROS nodes and topics used in the project. 

![alt text](./imgs/final-project-ros-graph-v2.png?raw=true "ROS Nodes Description")
#### DBW Node
 This package contains the files that are responsible for control of the vehicle. It publishes the throttle, brake, and steering commands. To minimise jerk the PID controller gets resetted when manual driver takes over.
#### Waypoint Follower
A package containing code from Autoware which publishes target vehicle linear and angular velocities in the form of twist commands. 
#### Waypoint Updater
The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. 
#### Traffic Light Detection
This node handels traffic light detection and a traffic light classification. It publishes also the locations to stop for red traffic lights.

Further documentation can be found at https://github.com/AndysDeepAbstractions/Early_Birds_CarND-Capstone/blob/80dcb5d297cb728799032509c83b9a58d4e42620/TrafficLight/README.md


---
## Acknowledgments

---
## Known Issues

Since github has a 100MB file limit the traffic light detection model for some earlier versions has to be downloaded from:
https://drive.google.com/open?id=19ehIoc0koBII2urasg3kHf9EOAiI2bbo
and needs to be placed in
./Early_Birds_CarND-Capstone/ros/src/tl_detector/light_classification/frozen_models/

---
## Conclusions

- Udacity Review and Clara test drive upcoming.

---
## Installation
#### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

#### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

#### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git # for the initial repo
# or 
git clone https://github.com/AndysDeepAbstractions/Early_Birds_CarND-Capstone # for the Early_Birds Project repo
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

#### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
