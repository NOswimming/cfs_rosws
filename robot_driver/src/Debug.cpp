#include "Debug.h"
#include "Vector2Int.h"
#include <iostream>
#include <fstream>

using namespace std;

Debug::Debug()
{
	init(true);
}

Debug::Debug(bool enable)
{
	init(enable);	
}

void Debug::init(bool enable)
{
	if (enable)
	{
		PrintStateChange = true;
		WriteMapUpdateToFile = true;
		PrintCurrentPose = false;
		PrintScannedArray = false;
		PrintNewTarget = true;
	}
	else
	{
		// If diabled then turn off all output
		PrintStateChange = false;
		WriteMapUpdateToFile = false;
		PrintCurrentPose = false;
		PrintScannedArray = false;
		PrintNewTarget = false;
	}
}

////////// DEBUGGING FUNCTIONS //////////

/**
 * Prints the tiled map.
 */
void DEBUG_printTiledMap(int tiledMap[], Vector2Int tiledMapSize)
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
 * Prints the navigation map.
 */
void DEBUG_printNavigationMap(int navigationMap[], Vector2Int navigationMapSize)
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

/**
 * Prints the 2D navigation map.
 */
void DEBUG_printNavigationMap2d(int navigationMap2d[][80], Vector2Int navigationMapSize)
{
	// Open a file to print to
	char filename[] = "/afs/ec.auckland.ac.nz/users/h/z/hzha321/unixhome/DEBUG_navigation_map_2d.txt";
	ofstream  myfile(filename);
	myfile << "2D navigation map Output:\n";

	char buf[5];

	for (int y = 0; y < navigationMapSize.y; y++)
	{
		for (int x = 0; x < navigationMapSize.x; x++)
		{
			snprintf (buf, 7, "%02d,", navigationMap2d[x][y]);
			myfile << buf;
		}

		myfile << "\n";
	}

	myfile.close();

	ROS_INFO("\nDEBUG Output 2D navigation map to file: %s", filename);

}


void DEBUG_printStepsToTarget(int steps[][2], int finalStep)
{
	// Open a file to print to
	char filename[] = "/afs/ec.auckland.ac.nz/users/h/z/hzha321/unixhome/DEBUG_steps.txt";
	ofstream  myfile(filename);
	myfile << "Steps to target Output:\n";

	char buf[5];

	for (int i = 0; i < finalStep; i++)
	{
		snprintf (buf, 7, "%02d %02d\n", steps[i][0], steps[i][1]);
		myfile << buf;

	}

	myfile.close();

	ROS_INFO("\nDEBUG Output steps to target to file: %s", filename);
}
