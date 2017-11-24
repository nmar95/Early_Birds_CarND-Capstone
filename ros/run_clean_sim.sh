#enc
source /opt/ros/kinetic/setup.bash

#cd ros

# clean
rm -rf devel
rm -rf build
catkin_make clean
rosclean

# run
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch

