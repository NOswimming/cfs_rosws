#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include "Debug.h"
#include "Vector2Int.h"
#include "Vector2Float.h"
#include "RobotHelperFunctions.cpp"
#include "Mapping.cpp"

using namespace std;



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
int tiledMap [6400];

// Navigation map
Vector2Int navigationMapSize(tiledMapSize.x, tiledMapSize.y);
int navigationMap [6400];

// Mapping 2D
int navigationMap2d[80][80];

// Path
Vector2Float currentGoal (0,0);
int currentStep = 0;
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

void initializeRobot()
{
	// Initialize all values to unknown
	std::fill_n(tiledMap, 6400, -1); 
	std::fill_n(navigationMap, 6400, -1);
}

void setNextState(int state, bool callEnterState)
{
	nextState = state;
	nextCallEnterState = callEnterState;
}

void setRobotMovementWithTolerance(float linearSpeed, float rotationalSpeed, float laserScanTolerance)
{
	velocityCommand.linear.x = linearSpeed;
	velocityCommand.angular.z = rotationalSpeed;
	scanTolerance = laserScanTolerance;
}

void setRobotMovement(float linearSpeed, float rotationalSpeed)
{
	setRobotMovementWithTolerance(linearSpeed, rotationalSpeed, scanTolerance);
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
	// Get current grid position for navigation array
	Vector2Float globalPosition(currentPose.position.x, currentPose.position.y);
	Vector2Int gridPosition(0,0);
	gridPosition = getGridPositionForOdom(globalPosition, mapOffset, tiledMapResolution);
	
	if (debug.PrintNewTarget)
	{
		ROS_INFO("Grid position: %d %d", gridPosition.x, gridPosition.y);
	}
	
	algorithmPos start;
	retData data;
	start.x = gridPosition.x;
	start.y = gridPosition.y;
	
	convertArrayTo2D(navigationMap, navigationMap2d, navigationMapSize);
	
	if (debug.PrintNewTarget)
	{
		DEBUG_printNavigationMap2d(navigationMap2d, navigationMapSize);
	}
	
	
	algorithmPos target = findTarget(start,navigationMap2d);
	
	
	if (debug.PrintNewTarget)
	{
		ROS_INFO("New target: %d %d", target.x, target.y);
	}
	
	CalculateMap (start, target, navigationMap2d, data, 0);
	
	
	ROS_INFO("Final step number: %d", finalStepNumber);
	ROS_INFO("Final step 1: %d %d", finalSteps[0][0], finalSteps[0][1]);
	DEBUG_printStepsToTarget(finalSteps, finalStepNumber);
	
}

void pathToCurrentGoal(const geometry_msgs::Pose &pose)
{
	// get grid position
	Vector2Int gridPosition (finalSteps[currentStep][0], finalSteps[currentStep][1]);
	// get global position
	currentGoal = getOdomPositionForGrid(gridPosition, mapOffset, tiledMapResolution);
	
	Vector2Float currentOdomPosition (currentPose.position.x, currentPose.position.y);
	float currentOrientation = currentPose.orientation.z;
	
	float desiredOrientation =  getDesiredOrientation(currentOdomPosition, currentGoal);
	
	
	if(currentOrientation < desiredOrientation)
	{
		setRobotMovement(0.00, 0.6);
	}
	
	if(currentOrientation > desiredOrientation)
	{
		setRobotMovement(0.00, -0.6);
	}
	
	
}

////////// SCANNING FUNCTIONS //////////

void newScan(float currentOrientation)
{
	setRobotMovementWithTolerance(0, -0.6, 0.1);
	scanStartOrientation = currentOrientation;
	scanned[0] = 0;
	scanned[1] = 0;
	scanned[2] = 0;
	scanned[3] = 0;
}

void scanArea(float currentOrientation)
{
	// Set to be stationary and turn
	setRobotMovement(0, -0.6);

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
		//setNextState(STATE_SCANNING, true);
		// TODO: chnage
		setNextState(STATE_PATHING, true);
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
