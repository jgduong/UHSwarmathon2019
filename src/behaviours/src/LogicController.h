#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unordered_map>
#include <set>
#include <math.h>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>

using namespace std;

#ifndef LOGICCONTROLLER_H
#define LOGICCONTROLLER_H

#include "Controller.h"
#include "SpiralSearchController.h"
/*
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"
#include "ObstacleController.h"
#include "DriveController.h"
#include "RangeController.h"
#include "ManualWaypointController.h"
*/
#include <vector>
#include <queue>

using namespace std;

enum States{
  SPIRAL_SEARCH = 0,
  AVOID_OBSTACLE,
  PICKUP,
  DROPOFF,
  FIND_SPIRAL_EDGE
};
/*
enum actions {
  stop = 0,
  driveStraight,
  rotate
};
*/


struct wheels {
  float left;
  float right;
};

float calcDistance(float curX, float curY, float goalX, float goalY) {
	float dist = sqrt( (goalX - curX)*(goalX - curX) + (goalY - curY)*(goalY - curY) );
	return dist;
}

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
//extern Swarmie thisSwarmie;

//typedef struct swarmie Swarmie;

class LogicController {
  private:
    SpiralSearchController spiralSearchController;
  
  public: 
	float currX; 
	float currY;
	float currTheta;
    	float initialPosX;
    	float initialPosY;
    	float startingTheta;
    	float centerOffsetX;
    	float centerOffsetY;
    	int prevState;
    	int currState;
    	struct wheels Wheels;
    	//priority_queue<Controller> ControllerQueue;
  
    	LogicController() {}
  
    	LogicController(float initialX, float initialY, float initialTheta) {
	    
		prevState = 0;
		currState = 0;

		initialPosX = initialX;
		initialPosY = initialY;
		startingTheta = initialTheta;
		currX = initialX;
		currY = initialY;
		currTheta = initialTheta;

		Wheels.left = 0.0;
		Wheels.right = 0.0;
    	}

	  void DoWork(int state) {
	    /*
	    Controller *currController;
	    for (int i = ControllerQueue.size(); i > 0; i--) {
		currController = ControllerQueue.pop();
		currController.DoWork();
	    }
	    */
	    if (state == SPIRAL_SEARCH) {
	      spiralSearchController.DoWork(currX, currY, currTheta);
	    }
	    else if (state == AVOID_OBSTACLE) {
	      //ObstacleController.DoWork();
	    }
	    else if (state == PICKUP) {
	      //PickupController.DoWork();
	    }
	    else if (state == DROPOFF) {
	      // DropoffController.DoWork();
	    }
	    else if (state == FIND_SPIRAL_EDGE) {
	      //FindEdgeController.DoWork();
	    }
	  }

	  void updateData(float x, float y, float theta) {
	    	currX = x;
	    	currY = y;
	    	currTheta = theta;
		spiralSearchController.updateData(x, y, theta);
	  }

	  void setCenterOffset(float x, float y) {
	    	centerOffsetX = x;
	    	centerOffsetY = y;
		spiralSearchController.setCenterOffset(x, y);
	  }

	  struct wheels InitialRotate() {
			//Rotate to starting position...
		  float ninetyRotate = currTheta;
		  cout << "Current theta is: " << currTheta << endl;
			/*
		  std_msgs::Float32MultiArray myCoordinate;
			myCoordinate.layout.dim.push_back(std_msgs::MultiArrayDimension());
			myCoordinate.layout.dim[0].size = 2;
			myCoordinate.layout.dim[0].stride = 1;
			myCoordinate.layout.dim[0].label = "poop";

			myCoordinate.data.push_back(normalizedValue(thisSwarmie->currX+centerOffsetX));
			myCoordinate.data.push_back(normalizedValue(thisSwarmie->currY+centerOffsetY));

			visitedLocations[myCoordinate.data[0]].insert(myCoordinate.data[1]);

			visitedLocationsPublisher.publish(myCoordinate);
		  */
			cout << "step 2: rotating 90 degrees left..." << endl;

			geometry_msgs::Point tempLocal;

			float turnSize = 1.5;
			bool exceedMag = false;

			//if the ninety degree turn crosses the pi line, this is a special condition
			if (abs(startingTheta + turnSize) >= 3.14159)
			{
				exceedMag = true;
			}
			cout << "exceed magnitude value is " << exceedMag << endl;
			if (exceedMag)
			{
				float desiredTheta = 0.0;

				desiredTheta = -3.142 + (startingTheta - turnSize);
				if (currTheta >= desiredTheta && currTheta < 0.0)
				{
					//sendDriveCommand(0.0, 0.0);
					Wheels.left = 0.0;
					Wheels.right = 0.0;
					//rotateBool = false;
					//hardcodedPop = true;
					//initialMove = true;
					//step = 1;
					initialPosX = currX;
					initialPosY = currY;

					cout << "done rotating" << endl;
				}
				else {
					//sendDriveCommand(-30.0, 30.0);
					Wheels.left = -30.0;
					Wheels.right = 30.0;
					cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
				}



			}
			else
			{	//ninetyRotate = current theta
				//if the diff between the current theta and starting theta is >= 90 degrees, stop
				if (abs(ninetyRotate - startingTheta) >= 1.5)
				{
					//sendDriveCommand(0.0, 0.0);
					Wheels.left = 0.0;
					Wheels.right = 0.0;
					//    rotateBool = false;
					//hardcodedPop = true;
					//initialMove = true;
					//GPSCenter = true;
					//step = 1;
					initialPosX = currX;
					initialPosY = currY;
					cout << "done rotating" << endl;

				 }	//else, turn right
				 else {
					//sendDriveCommand(-30.0, 30.0);
					Wheels.left = -30.0;
					Wheels.right = 30.0;
				 }

		}
		return Wheels;
	  }
};

#endif // LOGICCONTROLLER_H
