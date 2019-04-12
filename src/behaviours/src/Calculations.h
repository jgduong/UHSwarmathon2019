#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <math.h>

float normalizedValue(float x)
{
	bool isNegative;
	if (x < 0)
	{
		isNegative = true;
	}
	else {
		isNegative = false;
	}
	x = fabs(x);
	float temp = x * 100;
	int val = round(temp);
	
	if ((val % 25) > 13)
	{
		int n = (val/25) + 1;
		if (isNegative)
		{
			return (-1)*n*0.25;
		}
		else
		{
			return n*0.25;
		}
			
	}
	else
	{
		int n = (val/25);
		if (isNegative)
		{
			return (-1)*n*0.25;
		}
		else
		{
			return n*0.25;
		}
			
	}
}

float calcDistance(float curX, float curY, float goalX, float goalY) {
	float dist = sqrt( (goalX - curX)*(goalX - curX) + (goalY - curY)*(goalY - curY) );
	return dist;
}

#endif
