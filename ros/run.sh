source /opt/ros/kinetic/setup.bash

#cd ros

rm -rf devel
rm -rf build
catkin_make clean

catkin_make
source devel/setup.sh
roslaunch launch/styx.launch

