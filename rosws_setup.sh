#!/bin/bash

# Creates a new ROS workspace in folder rosws with new ROS robot_driver package

echo "///////////////////////////////"
echo "///// SETUP ROS WORKSPACE /////"
echo "///////////////////////////////"

echo "Removing rosws folder"
rm -rf ~/rosws || { echo '///// rm -rf ~/rosws FAILED! /////' ; exit 1; }

echo "Creating a new ROS workspace"

# souce on computer ros installation
source /opt/ros/indigo/setup.bash || { echo '///// source /opt/ros/indigo/setup.bash FAILED! /////' ; exit 1; }

# make directory
mkdir -p ~/rosws/src || { echo '///// mkdir -p ~/rosws/src FAILED! /////' ; exit 1; }

# initialise ROS workspace
cd ~/rosws/src || { echo '///// cd ~/rosws/src FAILED! /////' ; exit 1; }
catkin_init_workspace || { echo '///// catkin_init_workspace FAILED! /////' ; exit 1; }

echo "Creating robot_driver ROS package"

# creates robot_driver package
cd ~/rosws/src || { echo '///// cd ~/rosws/src FAILED! /////' ; exit 1; }
catkin_create_pkg robot_driver std_msgs roscpp || { echo '///// catkin_create_pkg FAILED! /////' ; exit 1; }

# create the pioneerLaser.cpp file and copy content to it
cat ~/cfs_rosws/pioneerLaser.cpp >> ~/rosws/src/robot_driver/src/pioneerLaser.cpp || { echo '///// concat pioneerLaser.cpp FAILED! /////' ; exit 1; }

# create the launch files and copy content to them
mkdir -p ~/rosws/src/robot_driver/launch || { echo '///// mkdir -p ~/rosws/src/robot_driver/launch FAILED! /////' ; exit 1; }
cat ~/cfs_rosws/robot.launch >> ~/rosws/src/robot_driver/launch/robot.launch || { echo '///// concat robot.launch FAILED! /////' ; exit 1; }
cat ~/cfs_rosws/stage.launch >> ~/rosws/src/robot_driver/launch/stage.launch || { echo '///// concat stage.launch FAILED! /////' ; exit 1; }
mkdir -p ~/rosws/src/robot_driver/world || { echo '///// mkdir -p ~/rosws/src/robot_driver/world FAILED! /////' ; exit 1; }
cat ~/cfs_rosws/myworld.world >> ~/rosws/src/robot_driver/world/myworld.world || { echo '///// concat myworld.world FAILED! /////' ; exit 1; }

# add lines to CMakelists.txt
cat <<EOT >> ~/rosws/src/robot_driver/CMakeLists.txt
add_executable( pioneer_laser_node src/pioneerLaser.cpp)
target_link_libraries(pioneer_laser_node \${catkin_LIBRARIES})
add_dependencies(pioneer_laser_node \${catkin_EXPORTED_TARGETS})
EOT

# build the new package
cd ~/rosws || { echo '///// cd ~/rosws  FAILED! /////' ; exit 1; }
catkin_make || { echo '///// catkin_make FAILED! /////' ; exit 1; }

# source to the workspace with the new package
source ~/rosws/devel/setup.bash || { echo '///// source ~/rosws/devel/setup.bash FAILED! /////' ; exit 1; }

echo "////////////////////////////////////////"
echo "///// FINISHED SETUP SUCCESSFULLY! /////"
echo "////////////////////////////////////////"
