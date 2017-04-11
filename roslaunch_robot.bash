#!/bin/bash

# Runs ROS for the pioneer robot with a launch file

echo "/////////////////////////////"
echo "///// RUN ROS FOR ROBOT /////"
echo "/////////////////////////////"

echo "Copying file content from the ~/cfs_rosws directory"

# remove directory contents and copy new file content from the ~/cfs_rosws directory
rm -r ~/rosws/src/robot_driver/src/* || { echo '///// removing src files FAILED! /////' ; exit 1; }
cp -Trv ~/cfs_rosws/robot_driver/src/ ~/rosws/src/robot_driver/src/ || { echo '///// copy src files FAILED! /////' ; exit 1; }
rm -r ~/rosws/src/robot_driver/launch/* || { echo '///// removing launch files FAILED! /////' ; exit 1; }
cp -Trv ~/cfs_rosws/robot_driver/launch/ ~/rosws/src/robot_driver/launch/ || { echo '///// copy launch files FAILED! /////' ; exit 1; }
rm -r ~/rosws/src/robot_driver/world/* || { echo '///// removing world files FAILED! /////' ; exit 1; }
cp -Trv ~/cfs_rosws/robot_driver/world/ ~/rosws/src/robot_driver/world/ || { echo '///// copy world files FAILED! /////' ; exit 1; }

# build the new package
cd ~/rosws || { echo '///// cd ~/rosws FAILED! /////' ; exit 1; }
source ~/rosws/devel/setup.bash || { echo '///// source ~/rosws/devel/setup.bash FAILED! /////' ; exit 1; }
catkin_make || { echo '///// catkin_make FAILED! /////' ; exit 1; }

# source to the workspace with the new package
source ~/rosws/devel/setup.bash || { echo '///// source ~/rosws/devel/setup.bash FAILED! /////' ; exit 1; }

# run ROS for the robot with roslaunch
roslaunch robot_driver robot.launch || { echo '///// roslaunch FAILED! /////' ; exit 1; }
