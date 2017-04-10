#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
using namespace std;

////////// CLASSES //////////

class Vector2Int
{
public:
	int x, y;
	Vector2Int(int X, int Y)
	{
		x = X;
		y = Y;
	}
	;
};

class Vector2Float
{
public:
	float x, y;
	Vector2Float(float X, float Y)
	{
		x = X;
		y = Y;
	}
	;
};

////////// VARIABLES //////////

// Debug output
bool DEBUG_statechange = true;

geometry_msgs::Twist velocityCommand;
geometry_msgs::Pose currentPose;

float prevX, prevY, prevOdom;
int debugLoopCount = 0;

float mapResolution = 0.05;
Vector2Int mapSize(400, 400);
Vector2Float mapOffset(-10, -10);

// Tiled map
float tiledMapResolution = 0.25;
int tiledMapScaling = tiledMapResolution / mapResolution;
Vector2Int tiledMapSize(mapSize.x / tiledMapScaling, mapSize.y / tiledMapScaling);
int tiledMap[6400] =
{ -1 }; // Initialize all values to unknown

// Navigation map
Vector2Int navigationMapSize(tiledMapSize.x, tiledMapSize.y);
int navigationMap[6400] =
{ -1 }; // Initialize all values to unknown

// Path
Vector2Float currentGoal (0,0);
//Vector2Float pathPoints[3] { { 1, 1 }, { 2, 2 }, { 3, 3 } };

// Scan
float scanStartOrientation = 0;
int scanned[4] = {0,0,0,0};
float scanTolerance = 0.01;

// States
int previousState = 0;
int currentState = 0;
int STATE_START = 0;
int STATE_OBJECTAVOIDANCE = -1;
int STATE_PATHING = 1;
int STATE_SCANNING = 2;

////////// HELPER FUNCTIONS //////////

/**
 * Gets the average laser data range for a given set of indices inclusive.
 *
 * @param laserScanData - The laser scan data to get the ranges for.
 * @param startIndex - The first index to get the data for.
 * @param finishIndex - The last index to get the data for.
 * @return The average laser data range.
 */
float averageRange(const sensor_msgs::LaserScan::ConstPtr& laserScanData, int startIndex, int finishIndex)
{
	float sum = 0;

	for (int i = startIndex; i <= finishIndex; i++)
	{
		// If value is not a number or is larger than the maximum range then add the maximum range
		if (isnan(laserScanData->ranges[i]) || laserScanData->ranges[i] > laserScanData->range_max)
		{
			sum += laserScanData->range_max;
		}
		// If value is smaller than the minimum range then add the minimum range
		else if (laserScanData->ranges[i] < laserScanData->range_min)
		{
			sum += laserScanData->range_min;
		}
		// If value is within the acceptable range then add the value
		else
		{
			sum += laserScanData->ranges[i];
		}
	}
	int range = finishIndex - startIndex;
	return sum / range;
}

Vector2Int getGridPositionForOdom(Vector2Float &odomPosition, Vector2Float &gridOffset, float gridResolution)
{
	int x = (odomPosition.x - gridOffset.x) / gridResolution;
	int y = (odomPosition.y - gridOffset.y) / gridResolution;
	return Vector2Int(x, y);
}

Vector2Float getOdomPositionForGrid(Vector2Int &gridPosition, Vector2Float &gridOffset, float gridResolution)
{
	float x = gridPosition.x * gridResolution + gridOffset.x;
	float y = gridPosition.y * gridResolution + gridOffset.y;
	return Vector2Float(x, y);
}

/**
 * Sums the values in the area of an occupancy grid data array.
 *
 * @param map - Occupancy grid data.
 * @param offset - Starting position for the summation.
 * @param size - Size of the array to sum.
 * @return The sum of the array values in the area.
 */
int arrayAreaSum(const nav_msgs::OccupancyGrid::ConstPtr &map, Vector2Int &offset, Vector2Int &size)
{
	int sum = 0;
	for (int y = offset.y; y < offset.y + size.y; y++)
	{
		if (y >= 0 && y < map->info.height)
		{
			// Within map bounds
			for (int x = offset.x; x < offset.x + size.x; x++)
			{

				if (x >= 0 && x < map->info.width)
				{
					// Within map bounds
					int value = map->data[map->info.width * y + x];
					if (!isnan(value))
					{
						sum += value;
					}
				}
				else
				{
					// Outside map bounds
					sum += 1000;
					if (x >= map->info.width)
					{
						break;
					}
				}
			}
		}
		else
		{
			// Outside map bounds
			sum += 1000;
			if (y >= map->info.height)
			{
				break;
			}
		}
	}
	return sum;
}

////////// DEBUGGING FUNCTIONS //////////

/**
 * Prints the tiled map.
 */
void DEBUG_printTiledMap()
{
	// Open a file to print to
	char filename[] = "/home/robot/DEBUG_tiled_map.txt";
	ofstream  myfile(filename);
	myfile << "Tiled Map Output:\n";

	char buf[5];

	for (int y = 0; y < tiledMapSize.y; y++)
	{
		for (int x = 0; x < tiledMapSize.x; x++)
		{
			snprintf (buf, 7, "%04d,", tiledMap[tiledMapSize.x * y + x]);
			myfile << buf;
		}

		myfile << "\n";
	}

	myfile.close();

	ROS_INFO("\nDEBUG Output tiled map to file: %s", filename);

}

/**
 * Prints the tiled map.
 */
void DEBUG_printNavigationMap()
{
	// Open a file to print to
	char filename[] = "/home/robot/DEBUG_navigation_map.txt";
	ofstream  myfile(filename);
	myfile << "Tiled Map Output:\n";

	char buf[5];

	for (int y = 0; y < navigationMapSize.y; y++)
	{
		for (int x = 0; x < navigationMapSize.x; x++)
		{
			snprintf (buf, 7, "%02d,", navigationMap[navigationMapSize.x * y + x]);
			myfile << buf;
		}

		myfile << "\n";
	}

	myfile.close();

	ROS_INFO("\nDEBUG Output navigation map to file: %s", filename);

}


////////// STATE FUNCTIONS //////////

void getStateName(int state, char* outStr)
{
	if (state == STATE_START)
	{
		strcpy(outStr, "START");
		return;
	}
	if (state == STATE_PATHING)
	{
		strcpy(outStr, "PATHING");
		return;
	}
	if (state == STATE_OBJECTAVOIDANCE)
	{
		strcpy(outStr, "OBJECT AVOIDANCE");
		return;
	}
	if (state == STATE_SCANNING)
	{
		strcpy(outStr, "SCANNING");
		return;
	}
	strcpy(outStr, "UNKNOWN STATE");
}


void setCurrentState(int state)
{
	previousState = currentState;
	currentState = state;
	
	if (DEBUG_statechange)
	{
		char previousStateName[20];
		getStateName(previousState, previousStateName);
		char currentStateName[20];
		getStateName(currentState, currentStateName);
		
		ROS_INFO("\nState changed from %s to %s", previousStateName, currentStateName);
	}
}

void gotoPreviousState()
{
	setCurrentState(previousState);
}



////////// MAPPING FUNCTIONS //////////

/**
 * Populates the tiled map by reducing the occupancy grid to a lower resolution and summing the values.
 *
 * @param map - The gmapping occupancy grid to reduce.
 */
void populateTiledMap(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	Vector2Float globalPosition(0, 0);
	Vector2Int offset(0, 0);
	Vector2Int size(tiledMapScaling, tiledMapScaling);
	Vector2Int position(0, 0);

	for (int y = 0; y < tiledMapSize.y; y++)
	{
		for (int x = 0; x < tiledMapSize.x; x++)
		{
			position.x = x;
			position.y = y;
			// Get global position based on local tiled map grid position
			globalPosition = getOdomPositionForGrid(position, mapOffset, tiledMapResolution);
			// Get offset for the occupancy grid
			offset = getGridPositionForOdom(globalPosition, mapOffset, mapResolution);
			// Sum the area
			int areaSum = arrayAreaSum(map, offset, size);
			// Set the tile to the sum
			tiledMap[tiledMapSize.x * y + x] = areaSum;
		}
	}
}

int getTiledMapValue(Vector2Int &coordinates)
{
	if (coordinates.x < 0 || coordinates.x >= tiledMapSize.x)
	{
		return 1000;
	}

	if (coordinates.y < 0 || coordinates.y >= tiledMapSize.y)
	{
		return 1000;
	}

	return tiledMap[tiledMapSize.x * coordinates.y + coordinates.x];
}

/**
 * Populates the navigation map by using the tiled map grid and overlapping the tiles.
 */
void populateNavigationMap()
{
	Vector2Int positionTopRight(0, 0);
	Vector2Int positionTopLeft(0, 0);
	Vector2Int positionBottomRight(0, 0);
	Vector2Int positionBottomLeft(0, 0);

	for (int y = 0; y < navigationMapSize.y; y++)
	{
		for (int x = 0; x < navigationMapSize.x; x++)
		{
			// If the tile is not unknown then skip it
			if (navigationMap[navigationMapSize.x * y + x] != -1)
			{
				break;
			}

			positionTopRight.x = x;
			positionTopRight.y = y;

			positionTopLeft.x = x - 1;
			positionTopLeft.y = y;

			positionBottomRight.x = x;
			positionBottomRight.y = y - 1;

			positionBottomLeft.x = x - 1;
			positionBottomLeft.y = y - 1;

			// Get four tiles
			int topRight = getTiledMapValue(positionTopRight);
			int topLeft = getTiledMapValue(positionTopLeft);
			int bottomRight = getTiledMapValue(positionBottomRight);
			int bottomLeft = getTiledMapValue(positionBottomLeft);

			int value = 1;

			// If any of the tiles are occupied then treat the area as occupied
			if (topRight > 0 || topLeft > 0 || bottomRight > 0 || bottomLeft > 0)
			{
				value = 1;
			}
			// If any of the tiles are unknown then treat the area as unknown
			else if (topRight < 0 || topLeft < 0 || bottomRight < 0 || bottomLeft < 0)
			{
				value = -1;
			}
			else
			{
				value = 0;
			}

			navigationMap[navigationMapSize.x * y + x] = value;

		}
	}
}

////////// PATHING FUNCTIONS //////////

void pathToCurrentGoal(const nav_msgs::Odometry::ConstPtr &odom)
{
	// TODO
}

////////// SCANNING FUNCTIONS //////////

void newScan(float currentOrientation)
{
	scanStartOrientation = currentOrientation;
	scanned[0] = 0;
	scanned[1] = 0;
	scanned[2] = 0;
	scanned[3] = 0;
}

void scanArea(float currentOrientation)
{
	// Set to be stationary and turn
	velocityCommand.linear.x = 0;
	velocityCommand.angular.z = -0.6;
	
	
	// start orientation requires extra condition or will immediately be true
	if (currentOrientation > scanStartOrientation - scanTolerance && currentOrientation < scanStartOrientation + scanTolerance && scanned[0] == 1)
	{
		scanned[3] = 1;
	}
}


////////// CALLBACK FUNCTIONS //////////

/**
 * Callback function for subscription to the gmapping occupancy grid.
 * @param map - gmapping occupancy grid.
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	int width = map->info.width;
	int height = map->info.height;
	float infoX = map->info.origin.position.x;
	float infoY = map->info.origin.position.y;

	ROS_INFO("\nMap Size: %d, %d\nMap Origin: %f, %f", width, height, infoX, infoY);

	// Populate the tiled map by reducing the occupancy grid to a lower resolution and summing the values
	populateTiledMap(map);
	DEBUG_printTiledMap();

	populateNavigationMap();
	DEBUG_printNavigationMap();

	ROS_INFO("\nDATA: %f , %f", infoX, infoY);
}

/**
 * Callback function for subscription to the robot odometry.
 * @param odom - robot odometry message
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
	if (currentState == STATE_START)
	{
		newScan(odom->pose.pose.orientation.z);
		setCurrentState(STATE_SCANNING);
	}
	
	if (currentState == STATE_SCANNING)
	{
		scanArea(odom->pose.pose.orientation.z);
	}
	
	if (currentState == STATE_PATHING)
	{
		pathToCurrentGoal(odom);
	}

	currentPose = odom->pose.pose;

	float currentX = currentPose.position.x;
	float currentY = currentPose.position.y;
	float orientZ = currentPose.orientation.z;

ROS_INFO("\nDATA: %f, %f\nORIENT: %f",currentX,currentY,orientZ);
}

/**
 * Callback function for subscription to the robot laser scanner.
 * @param laserScanData - robot laser scan message.
 */
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	float currentX = currentPose.position.x;
	float currentY = currentPose.position.y;
	float orientZ = currentPose.orientation.z;

// Example of using some of the non-range data-types
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min) / (laserScanData->angle_increment);
	float leftRange = 0, rightRange = 0;
//Defined constants
	int j = 0;
//Left

	velocityCommand.linear.x = 0.1;
	velocityCommand.angular.z = 0;

	float leftVal = averageRange(laserScanData, rangeDataNum - 65, rangeDataNum - 1);
	float rightVal = averageRange(laserScanData, 0, 63);
	float midVal = averageRange(laserScanData, rangeDataNum / 2 - 32, rangeDataNum / 2 + 32);

// DEBUG INFORMATION
	if (debugLoopCount == 15)
	{
		//ROS_INFO("\nLaser Scan Data: Range Max: %f Range Min: %f", laserScanData->range_max, laserScanData->range_min);
		//ROS_INFO("\nTest Global odom: %f, %f\nORIENT: %f", currentX, currentY, orientZ);
		//ROS_INFO("\nlazers: %f, %f,  %f", leftVal, midVal, rightVal);
		debugLoopCount = 0;
	}
	debugLoopCount++;

//To avoid obstacles
	for (int j = 0; j < rangeDataNum; ++j) // Go through the laser data
	{
		if (laserScanData->ranges[j] < 0.25)
		{
			velocityCommand.linear.x = 0;
			if (laserScanData->ranges[0] > laserScanData->ranges[(int) rangeDataNum - 1])
			{
				velocityCommand.angular.z = -0.6;
			}
			else
			{
				velocityCommand.angular.z = 0.6;
			}
			return;

		}
	}

//F shaped intersection
	if (((rightVal > 60) || isnan(rightVal)) && (midVal > 60 || isnan(midVal)))
	{
		if (prevX == -1 && prevY == -1)
		{
			prevX = currentX;
			prevY = currentY;
			return;
		}
		else
		{
			if (((prevX - currentX) * (prevX - currentX) + (prevY - currentY) * (prevY - currentY)) > 3)
			{
				velocityCommand.linear.x = 0;
				ROS_INFO("\nTFshapeIntersection: %f, %f\nORIENT: %f", currentX, currentY, orientZ);
			}
		}
	}
	else
	{
		//Reset odom data
		prevX = -1;
		prevY = -1;
	}

// Set to stationary
/*	velocityCommand.linear.x = 0;
	velocityCommand.angular.z = 0;
*/

}

////////// MAIN FUNCTION //////////

/**
 * Main function for starting the pioneer laser node used by ROS.
 * Creates the publishers and subscribers for the pioneer laser node.
 */
int main(int argc, char **argv)
{
// Creates the ROS pioneer laser node
	ros::init(argc, argv, "pioneer_laser_node");
	ros::NodeHandle my_handle;

// Create ROS publishers for topics
	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);

// Create subscribers with callback functions for topics
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);
	ros::Subscriber map_sub_object = my_handle.subscribe("/map", 1, mapCallback);
	ros::Subscriber odom_sub_object = my_handle.subscribe("/RosAria/pose", 1, odomCallback);

// Publish the topics using the publishers at a set rate
// Publish rate (10 Hz)
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();

		// Publish the topics
		vel_pub_object.publish(velocityCommand);
	}

	return 0;
}
