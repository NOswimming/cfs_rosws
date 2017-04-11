#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include "Vector2Int.h"
#include "Vector2Float.h"

using namespace std;

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
