#ifndef _DEBUG_H
#define _DEBUG_H

class Debug
{
public:
	bool PrintStateChange;
	bool WriteMapUpdateToFile;
	bool PrintCurrentPose;
	bool PrintScannedArray;

	Debug();
	Debug(bool enable);
	
private:
	void init(bool enable);
};

#endif


