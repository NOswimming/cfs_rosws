#!/bin/bash

# Runs ROS for the pioneer robot with a launch file

echo "/////////////////////////////"
echo "///// RUN ROS FOR ROBOT /////"
echo "/////////////////////////////"

echo "Copying file content from the ~/cfs_rosws directory"

# copy file content from the ~/cfs_rosws directory
cp -T ~/cfs_rosws/pioneerLaser.cpp ~/rosws/src/robot_driver/src/pioneerLaser.cpp || { echo '///// copy pioneerLaser.cpp FAILED! /////' ; exit 1; }
cp -T ~/cfs_rosws/robot.launch ~/rosws/src/robot_driver/launch/robot.launch || { echo '///// copy robot.launch FAILED! /////' ; exit 1; }
cp -T ~/cfs_rosws/stage.launch ~/rosws/src/robot_driver/launch/stage.launch || { echo '///// copy stage.launch FAILED! /////' ; exit 1; }
cp -T ~/cfs_rosws/myworld.world ~/rosws/src/robot_driver/world/myworld.world || { echo '///// copy myworld.world FAILED! /////' ; exit 1; }

# build the new package
cd ~/rosws || { echo '///// cd ~/rosws FAILED! /////' ; exit 1; }
catkin_make || { echo '///// catkin_make FAILED! /////' ; exit 1; }

# source to the workspace with the new package
source ~/rosws/devel/setup.bash || { echo '///// cd ~/rosws FAILED! /////' ; exit 1; }

# run ROS for the robot with roslaunch
roslaunch robot_driver robot.launch || { echo '///// roslaunch FAILED! /////' ; exit 1; }
