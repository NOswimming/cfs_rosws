//============================================================================
// Name        : mapping.cpp
// Author      : tz
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdlib.h>
using namespace std;
#define ARRAYSIZE 80


struct algorithmPos{
	int y;
	int x;
};

struct retData {
	int retSteps[100][2];
	int retVisited[40][40];
};

int finalSteps[100][2];
int finalStepNumber = 0;



retData CalculateMap (algorithmPos start, algorithmPos target, int map[ARRAYSIZE][ARRAYSIZE], retData data,int stepCount){
	data.retSteps[stepCount][0] = start.x;
	data.retSteps[stepCount][1] = start.y;
	stepCount ++;
	if ((start.x == target.x) && (start.y == target.y)){
	    finalStepNumber = stepCount;
	    for (int k =0; k< stepCount;k++){
	        finalSteps[k][0] =  data.retSteps[k][0];
	        finalSteps[k][1] =  data.retSteps[k][1];
	    }
		return data;
	}
	int ys = start.y;
	int xs = start.x;
	int xt = target.x;
	int yt = target.y;
	data.retVisited[xs][ys] = 1;
	int HVals[4] = {999,999,999,999};
	int NextStep[4][2];
	//For a map, 0 is unblocked, 1 is blocked -1 is not explored
	if ((xs+1)<ARRAYSIZE ){
		if (map[xs+1][ys] != 1 && data.retVisited[xs+1][ys] == 0){
			HVals[0] = (xt-xs-1) * (xt-xs-1) + (yt-ys) * (yt-ys);
			NextStep[0][0] = xs+1;
			NextStep[0][1] = ys;
		}
	}
	if ((xs-1)>=0){
		if (map[xs-1][ys] != 1 && data.retVisited[xs-1][ys] == 0){
			HVals[1] = (xt-xs+1) * (xt-xs+1) + (yt-ys) * (yt-ys);
			NextStep[1][0] = xs-1;
			NextStep[1][1] = ys;
		}
	}
	if (ys+1 < ARRAYSIZE){
		if (map[xs][ys+1] != 1 && data.retVisited[xs][ys+1] == 0){
			HVals[2] = (xt-xs) * (xt-xs) + (yt-ys-1) * (yt-ys-1);
			NextStep[2][0] = xs;
			NextStep[2][1] = ys+1;
		}
	}
	if (ys-1 >=0){
		if (map[xs][ys-1] != 1 && data.retVisited[xs][ys-1] == 0){
			HVals[3] = (xt-xs) * (xt-xs) + (yt-ys+1) * (yt-ys+1);
			NextStep[3][0] = xs;
			NextStep[3][1] = ys-1;
		}
	}
	int minVal = HVals[0];
	int pos =0;
	algorithmPos minPos;
	for (int j=1;j<4;j++){
		if (HVals[j]< minVal){
			minVal = HVals[j];
			pos = j;
		}
	}
	algorithmPos newStart;
	//No Valid Option
	if (minVal ==999){
		if (stepCount == 1){
			finalStepNumber = -1;
			return data;
		}
		newStart.x = data.retSteps[stepCount-1][0];
		newStart.y = data.retSteps[stepCount-1][1];
		data.retSteps[stepCount][0] = 0;
		data.retSteps[stepCount][1] = 0;
		stepCount --;
	}
	else{
		newStart.x = NextStep[pos][0];
		newStart.y = NextStep[pos][1];
	}
	data = CalculateMap (newStart,  target, map, data, stepCount);


}

int capWithBounds (int a){
	if (a < 0 )
		return 0;
	if (a> (ARRAYSIZE-1))
		return (ARRAYSIZE-1);
	return a;
}

algorithmPos findTarget (algorithmPos start,int map[ARRAYSIZE][ARRAYSIZE]){
	algorithmPos target;
	int xs = start.y;
	int ys = start.x;
	int x1;
	int x2;
	int y1;
	int y2;
	for (int j =1; j<ARRAYSIZE;j++){
	    for (int i = 0; i < j + 1; i++)
	    {
	        x1 = capWithBounds(xs - j + i);
	        y1 = capWithBounds(ys - i);
	        if (map[x1][y1] == -1){
	        	target.x = x1;
	        	target.y = y1;
	        	return target;
	        }

	        x2 = capWithBounds(xs + j- i);
	        y2 = capWithBounds(ys + i);

	        if (map[x2][y2] == -1){
	        	target.x = x2;
	        	target.y = y2;
	        	return target;
	        }
	    }


	    for (int i = 1; i < j; i++)
	    {
	        x1 = capWithBounds(xs - i);
	        y1 = capWithBounds(ys + j - i);
	        if (map[x1][y1] == -1){
	        	target.x = x1;
	        	target.y = y1;
	        	return target;
	        }
	        // Check point (x1, y1)

	        x2 = capWithBounds(xs + j - i);
	        y2 = capWithBounds(ys - i);
	        if (map[x2][y2] == -1){
	        	target.x = x2;
	        	target.y = y2;
	        	return target;
	        }
	        // Check point (x2, y2)
	    }

	}
	//Return [0][0]
	return start;
}




/*
int main() {
	algorithmPos start = {} ;
	algorithmPos target = {} ;
	start.x = 0;
	start.y = 3;
	target.x =3;
	target.y =2;
	int map[4][4] = {{0,1,1,1},{0,1,1,1},{0,0,0,1},{1,1,0,1}};
	retData data;
	CalculateMap (start,  target, map, data, 0);
	if (finalStepNumber == -1){
		 cout <<"no valid path "<< endl;

	}
	for (int k =0; k< finalStepNumber;k++){
	    cout << finalSteps[k][0] << endl;
	    cout << finalSteps[k][1] << endl;
	}
    //Test findTarget
    int map1[4][4] = {{0,1,1,1},{-1,0,0,0},{1,1,0,0},{0,0,1,0}};
    algorithmPos start1 = {2,2};
    algorithmPos End1 = findTarget(start1,map1);
    //Should be (1,0)
    cout << End1.x << endl;
    cout << End1.y << endl;
    int map2[4][4] = {{-1,1,0,1},{-1,1,0,-1},{-1,1,1,1},{-1,-1,0,-1}};
    algorithmPos start2 = {1,2};
    algorithmPos End2 = findTarget(start2,map2);
    //Should be (0,3)
    cout << End2.x << endl;
    cout << End2.y << endl;
	return 0;
}
*/
