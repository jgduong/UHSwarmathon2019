#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <math.h>

float normalizedValue(float x)
{
	float temp = x * 100;
	int val = round(temp);
	
	if ((val % 25) > 13)
	{
		int n = (val/25) + 1;
		return n*0.25;
	}
	else
	{
		int n = (val/25);
		return n*0.25;
	}
}

float calcDistance(float curX, float curY, float goalX, float goalY) {
	float dist = sqrt( (goalX - curX)*(goalX - curX) + (goalY - curY)*(goalY - curY) );
	return dist;
}

bool isVisited(float x, float y, unordered_map<float, set<float>> &visitedLocations) {
	float normX = normalizedValue(x);
	float normY = normalizedValue(y);
     
	if (visitedLocations.find(normX) != visitedLocations.end()) {
        //x location exists in hashmap, check y coordinate
		if (visitedLocations[normX].find(normY) != visitedLocations[normX].end()) {
		//y location also exists, so this coordinate has been visited
		  	return true;
        	}
        	else {
            		return false;	
          	}
        }
        else {
          	return false;
        }
}

#endif
