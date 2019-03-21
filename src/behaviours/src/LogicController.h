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

#include "Swarmie.h"
#include "Tag.h"
#include "Calculations.h"
#include "Controller.h"
#include "SpiralSearchController.h"
#include "PickupController.h"
#include "DropoffController.h"
/*
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
  INIT = 0,
  SPIRAL_SEARCH,
  AVOID_OBSTACLE,
  PICKUP,
  DROPOFF,
  FIND_SPIRAL_EDGE
};


//extern Swarmie thisSwarmie;

//typedef struct swarmie Swarmie;


class LogicController {
  private:
    	SpiralSearchController spiralSearchController;
	PickupController pickupController;
	DropoffController dropoffController;
  
  public: 
	float currX; 
	float currY;
	float currTheta;
	float normX;
	float normY;
    	float initialPosX;
    	float initialPosY;
    	float startingTheta;
    	float centerOffsetX;
    	float centerOffsetY;
	float desiredTheta;
    	int prevState;
    	int currState;
	bool initVals = true;
	bool rotate = false;
    	Swarmie swarmie;
	unordered_map<float, set<float>> visitedLocations;
	
    	//priority_queue<Controller> ControllerQueue;
  
    	LogicController() {}
  
    	LogicController(float initialX, float initialY, float initialTheta) {
	    
		prevState = 0;
		currState = 0;

		initialPosX = normalizedValue(initialX);
		initialPosY = normalizedValue(initialY);
		startingTheta = initialTheta;
		currX = initialX;
		currY = initialY;
		normX = initialPosX;
		normY = initialPosY;
		
		currTheta = initialTheta;

		swarmie.left = 0.0;
		swarmie.right = 0.0;
		swarmie.wrist = 0;
		swarmie.finger = 0;
		//swarmie.initialized = false;
		
    	}

	  Swarmie DoWork(int state) {
		    currState = state;
		    if (state == SPIRAL_SEARCH) {
		      swarmie = spiralSearchController.DoWork(visitedLocations);
		    }
		    else if (state == AVOID_OBSTACLE) {
		      //ObstacleController.DoWork();
		    }
		    else if (state == PICKUP) {
		      swarmie = pickupController.DoWork();
		    }
		    else if (state == DROPOFF) {
		      swarmie = dropoffController.DoWork();
		    }
		    else if (state == FIND_SPIRAL_EDGE) {
		      //FindEdgeController.DoWork();
		    }
		    return swarmie;
	  }

	  void updateData(float x, float y, float theta) {
	    	currX = x;
	    	currY = y;
	    	currTheta = theta;
		  
		normX = normalizedValue(x);
		normY = normalizedValue(y);
		spiralSearchController.updateData(x, y, theta);
		pickupController.updateData(x, y);
		dropoffController.updateData(x, y, theta);
	  }
	
	void updateTags(vector<Tag> tags) {
		pickupController.updateTags(tags);
	}

	  void setCenterOffset(float x, float y) {
	    	centerOffsetX = x;
	    	centerOffsetY = y;
		spiralSearchController.setCenterOffset(x, y);
		dropoffController.setCenterOffset(x, y);
	  }
	
	    void populateMap() {
		//populates hashmap with points around the home base	
	      float x;
	      float y;
	      for (x = -1.50; x != 1.75; x+=0.25)
	      {
		for (y = -1.50; y != 1.75; y += 0.25)
		{
		  //initialPop.data.push_back(x);
		  //initialPop.data.push_back(y);

		  visitedLocations[x].insert(y);
		  //visitedLocationsPublisher.publish(initialPop);
		  //initialPop.data.clear();
		}
	      }
	      //hardcodedPop = false;
	      //mapTesting = true;
	      //rotateBool = true;
	      //initialMove = true;
	      //step = 10;

	      //startingTheta = currentLocationOdom.theta;
	    }

	
	Swarmie InitialRotate() {
		if (initVals) {
			desiredTheta = 	startingTheta + M_PI;
			if (desiredTheta > M_PI) {
				desiredTheta -= 2*M_PI;
			}
			initVals = false;
			rotate = true;
		}
		else if (rotate) {
			if (abs(desiredTheta - currTheta) <= 0.02) {
				cout << "Initial rotate complete" << endl;
				swarmie.left = 0.0;
				swarmie.right = 0.0;
			}
			else {
				swarmie.left = -40.0;
				swarmie.right = 40.0;
			}
		}
		
		return swarmie;
	}
	
	void addVisitedLocation(float x, float y) {
		if (currState == SPIRAL_SEARCH || currState == AVOID_OBSTACLE) {
			cout << "location added to hashmap" << endl;
			visitedLocations[normalizedValue(x)].insert(normalizedValue(y));
		}
	}
	
	Swarmie turnRight90() {
		spiralSearchController.updateData(currX, currY, currTheta);
		cout << "rotating right to begin spiral search..." << endl;
		float turnSize = -1.5;
		bool exceedMag = false;
		
		cout << "startingTheta is: " << startingTheta << endl;
		float ninetyRotate = currTheta;
		if (abs(startingTheta + turnSize) >= 3.142)
		{
			exceedMag = true;
		}
		cout << "exceed magnitude value is " << exceedMag << endl;
		if (exceedMag)
		{
			float desiredTheta = 0.0;

			desiredTheta = 3.142 + (startingTheta - turnSize);
			if (currTheta <= desiredTheta && currTheta > 0.0)
			{
				//sendDriveCommand(0.0, 0.0);
				swarmie.left = 0.0;
				swarmie.right = 0.0;
				//swarmie.initialized = true;
				cout << "done rotating: RIGHT 90" << endl;
				//step = 13;
				//initialMove = false;
				//mapTesting = true;
			}
			else {
				//sendDriveCommand(30.0, -30.0);
				swarmie.left = 30.0;
				swarmie.right = -30.0;
				cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
			}
			
			
			
		}
		else
		{
		      if (abs(ninetyRotate - startingTheta) >= 1.5)
		      {
			    //sendDriveCommand(0.0, 0.0); 
			      swarmie.left = 0.0;
			      swarmie.right = 0.0;
			      //swarmie.initialized = true;
			      cout << "done rotating: RIGHT 90" << endl;
			    //step = 13;
			     // initialMove = false;
			     // mapTesting = true;
		      }
		      else {
			    //sendDriveCommand(30.0, -30.0);
			    swarmie.left = 30;
			    swarmie.right = -30.0;
		      }

		}
		
		return swarmie;
	}

};

#endif // LOGICCONTROLLER_H
