#!/bin/bash

# Runs ROS for stage with a launch file

echo "/////////////////////////////"
echo "///// RUN ROS FOR STAGE /////"
echo "/////////////////////////////"

echo "Copying file content from the ~/cfs_rosws directory"

# copy file content from the ~/cfs_rosws directory
cp -T ~/cfs_rosws/pioneerLaser.cpp ~/rosws/src/robot_driver/src/pioneerLaser.cpp || { echo '///// cp pioneerLaser.cpp FAILED! /////' ; exit 1; }
cp -T ~/cfs_rosws/robot.launch ~/rosws/src/robot_driver/launch/robot.launch || { echo '///// cp robot.launch FAILED! /////' ; exit 1; }
cp -T ~/cfs_rosws/stage.launch ~/rosws/src/robot_driver/launch/stage.launch || { echo '///// cp stage.launch FAILED! /////' ; exit 1; }
cp -T ~/cfs_rosws/myworld.world ~/rosws/src/robot_driver/world/myworld.world || { echo '///// cp myworld.world FAILED! /////' ; exit 1; }

# build the new package
cd ~/rosws || { echo '///// cd ~/rosws FAILED! /////' ; exit 1; }
source ~/rosws/devel/setup.bash || { echo '///// source ~/rosws/devel/setup.bash FAILED! /////' ; exit 1; }
catkin_make || { echo '///// catkin_make FAILED! /////' ; exit 1; }

# source to the workspace with the new package
source ~/rosws/devel/setup.bash || { echo '///// source ~/rosws/devel/setup.bash FAILED! /////' ; exit 1; }

# run ROS for the robot with roslaunch
roslaunch robot_driver stage.launch || { echo '///// roslaunch FAILED! /////' ; exit 1; }
