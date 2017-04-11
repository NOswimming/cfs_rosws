#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
using namespace std;

// My included files
#include "Debug.cpp"
#include "Vector2Int.cpp"
#include "Vector2Float.cpp"

////////// VARIABLES //////////
Debug debug(true);




geometry_msgs::Twist velocityCommand;
geometry_msgs::Pose currentPose;

float prevX, prevY, prevOdom;
int debugLoopCount = 0;

float laserScanTolerance = 0.25;

float mapResolution = 0.05;
Vector2Int mapSize (400, 400);
Vector2Float mapOffset (-10, -10);

// Tiled map
float tiledMapResolution = 0.25;
int tiledMapScaling = tiledMapResolution / mapResolution;
Vector2Int tiledMapSize(mapSize.x / tiledMapScaling, mapSize.y / tiledMapScaling);
int tiledMap [6400] =
{ -1 }; // Initialize all values to unknown

// Navigation map
Vector2Int navigationMapSize(tiledMapSize.x, tiledMapSize.y);
int navigationMap [6400] =
{ -1 }; // Initialize all values to unknown

// Path
Vector2Float currentGoal (0,0);
//Vector2Float pathPoints[3] { { 1, 1 }, { 2, 2 }, { 3, 3 } };

// Scan
float scanStartOrientation = 0;
int scanned[4] = {0,0,0,0};
float scanTolerance = 0.1;

// State variables
int previousState = 0;
int currentState = 0;
int nextState = 0;
int nextCallEnterState = 0;

// STATES
int STATE_START = 0;
int STATE_OBJECTAVOIDANCE = -1;
int STATE_PATHING = 1;
int STATE_SCANNING = 2;


////////// HELPER FUNCTIONS //////////

void setNextState(int state, bool callEnterState)
{
	nextState = state;
	nextCallEnterState = callEnterState;
}

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

float wrapAngle(float angle)
{
	if (angle > 1)
	{
		return angle - 2;
	}

	if (angle < -1)
	{
		return angle + 2;
	}

	return angle;
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

void getNewGoal()
{
	// TODO
}

void pathToCurrentGoal(const geometry_msgs::Pose &pose)
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

	float scanOrientation90 = wrapAngle(scanStartOrientation + 0.5);
	float scanOrientation180 = wrapAngle(scanStartOrientation + 1);
	float scanOrientation270 = wrapAngle(scanStartOrientation + 1.5);

	// start orientation requires extra condition or will immediately be true
	if (currentOrientation > scanStartOrientation - scanTolerance && currentOrientation < scanStartOrientation + scanTolerance && scanned[2] == 1)
	{
		scanned[0] = 1;
	}

	if (currentOrientation > scanOrientation90 - scanTolerance && currentOrientation < scanOrientation90 + scanTolerance)
	{
		scanned[1] = 1;
	}

	if (currentOrientation > scanOrientation180 - scanTolerance && currentOrientation < scanOrientation180 + scanTolerance)
	{
		scanned[2] = 1;
	}

	if (currentOrientation > scanOrientation270 - scanTolerance && currentOrientation < scanOrientation270 + scanTolerance)
	{
		scanned[3] = 1;
	}

	// If all four axis have been scanned then change to path following
	if (scanned[0] + scanned[1] + scanned[2] + scanned[3] == 4)
	{
		setNextState(STATE_PATHING, true);
	}

	if (debug.PrintScannedArray)
	{
		ROS_INFO("\nScanned Array: %d %d %d %d \nScan Start Orientation: %f Current Orientation: %f", scanned[0], scanned[1], scanned[2], scanned[3], scanStartOrientation, currentOrientation);
	}


}

////////// STATE FUNCTIONS //////////

/**
 * Does any state specific setup when a state is entered
 */
void enterState(int state)
{
	if (state == STATE_START)
	{
		return;
	}
	if (state == STATE_PATHING)
	{
		getNewGoal();
		return;
	}
	if (state == STATE_OBJECTAVOIDANCE)
	{
		return;
	}
	if (state == STATE_SCANNING)
	{
		newScan(currentPose.orientation.z);
		return;
	}
	ROS_INFO("UNKNOWN STATE: %d", state);
}

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


void setCurrentState(int state, bool callEnterState)
{
	if (state == currentState)
	{
		// No change in state
		return;
	}

	// Set current and previous states
	previousState = currentState;
	currentState = state;

	// Call enter state for the new state
	if (callEnterState)
	{
		enterState(currentState);
	}

	if (debug.PrintStateChange)
	{
		char previousStateName[20];
		getStateName(previousState, previousStateName);
		char currentStateName[20];
		getStateName(currentState, currentStateName);

		ROS_INFO("\nState changed from %s to %s", previousStateName, currentStateName);
	}
}

void checkState()
{
	if (nextState != STATE_START) {
		setCurrentState(nextState, nextCallEnterState);
		nextState = STATE_START;
	}

	if (currentState == STATE_START)
	{
		setNextState(STATE_SCANNING, true);
	}

	if (currentState == STATE_SCANNING)
	{
		scanArea(currentPose.orientation.z);
	}

	if (currentState == STATE_PATHING)
	{
		pathToCurrentGoal(currentPose);
	}
}


////////// CALLBACK FUNCTIONS //////////

/**
 * Callback function for subscription to the gmapping occupancy grid.
 * @param map - gmapping occupancy grid.
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	// Populate the tiled map by reducing the occupancy grid to a lower resolution and summing the values
	populateTiledMap(map);
	// Update the naviagtion map based on the tiled map values
	populateNavigationMap();

	if (debug.WriteMapUpdateToFile)
	{
		ROS_INFO("\nMap Size: %d, %d\nMap Origin: %f, %f", map->info.width, map->info.height, map->info.origin.position.x, map->info.origin.position.y);
		DEBUG_printTiledMap();
		DEBUG_printNavigationMap();
	}
}

/**
 * Callback function for subscription to the robot odometry.
 * @param odom - robot odometry message
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
	// Update current pose
	currentPose = odom->pose.pose;

	checkState();


	if (debug.PrintCurrentPose)
	{
		ROS_INFO("CURRENT POSE\nPosition: %f, %f Orientation: %f", currentPose.position.x, currentPose.position.y, currentPose.orientation.z);
	}

}

/**
 * Callback function for subscription to the robot laser scanner.
 * @param laserScanData - robot laser scan message.
 */
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	// Array length
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min) / (laserScanData->angle_increment);

	float leftVal = averageRange(laserScanData, rangeDataNum - 65, rangeDataNum - 1);
	float rightVal = averageRange(laserScanData, 0, 63);
	float midVal = averageRange(laserScanData, rangeDataNum / 2 - 32, rangeDataNum / 2 + 32);

	//To avoid obstacles
	for (int j = 0; j < rangeDataNum; ++j) // Go through the laser data
	{
		if (laserScanData->ranges[j] < laserScanTolerance)
		{
			// If a laser scan is within the tolerance then move to object avoidance state
			setNextState(STATE_OBJECTAVOIDANCE, false);

			velocityCommand.linear.x = 0;
			if (rightVal > leftVal)
			{
				// Turn right
				velocityCommand.angular.z = -0.6;
			}
			else
			{
				// Turn left
				velocityCommand.angular.z = 0.6;
			}
			return;

		}
	}

	if (currentState == STATE_OBJECTAVOIDANCE)
	{
		setNextState(previousState, false);
	}

}
