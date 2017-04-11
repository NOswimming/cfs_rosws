#include "Debug.h"

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
	}
	else
	{
		// If diabled then turn off all output
		PrintStateChange = false;
		WriteMapUpdateToFile = false;
		PrintCurrentPose = false;
		PrintScannedArray = false;
	}
}
