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
#include "SpiralSearchController.h"
#include "PickupController.h"
#include "DropoffController.h"
#include "ObstacleController.h"

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
    	//SpiralSearchController spiralSearchController;
	//PickupController pickupController;
	//DropoffController dropoffController;
	//ObstacleController obstacleController;
  
  public: 
	SpiralSearchController spiralSearchController;
	PickupController pickupController;
	DropoffController dropoffController;
	ObstacleController obstacleController;
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
    	int prevState;
    	int currState;
	
	bool simulation = false;
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
		
    	}
	
	void setSimulationBool(bool isSim) {
		simulation = isSim;
		if (simulation) {
			cout << "Simulation is running" << endl;	
		}
		else {
			cout << "Physical is running" << endl;	
		}
		
		spiralSearchController.setSimulationBool(simulation);
		obstacleController.setSimulationBool(simulation);
		pickupController.setSimulationBool(simulation);
		dropoffController.setSimulationBool(simulation);
	}

	  Swarmie DoWork(int state, int prev) {
		  currState = state;
		  prevState = prev;
		    if (state == SPIRAL_SEARCH) {
		      swarmie = spiralSearchController.DoWork(visitedLocations);
		    }
		    else if (state == AVOID_OBSTACLE) {
		      swarmie = obstacleController.DoWork(prev, dropoffController.noForwards);
			    // VERY EXPERIMENTAL
		      /*if (prev == PICKUP && dropoffController.initCalc == true)
		      {
			      dropoffController.spiralX = currX;
			      dropoffController.spiralY = currY;
			      cout << "spiralX and spiralY have been set to: " << dropoffController.spiralX << ", " << dropoffController.spiralY << endl;
		      }*/
		    }
		    else if (state == PICKUP) {
		      swarmie = pickupController.DoWork();
		    }
		    else if (state == DROPOFF) {
		      /*if (prev == PICKUP && dropoffController.initCalc == true)
		      {
			      dropoffController.spiralX = currX;
			      dropoffController.spiralY = currY;
			      cout << "spiralX and spiralY have been set to: " << dropoffController.spiralX << ", " << dropoffController.spiralY << endl;
		      }*/
		      swarmie = dropoffController.DoWork(visitedLocations);
		    }

		    return swarmie;
	  }
	
	  void UpdateSonar(float left, float center, float right) {
		  
		  obstacleController.updateSonar(left, center, right);
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
	
	//void updateTags(float x, float y, float z) {
	void updateTags(vector<Tag> tags) {
		pickupController.updateTags(tags);
	}

	  void setCenterOffset(float x, float y) {
	    	centerOffsetX = x;
	    	centerOffsetY = y;
		spiralSearchController.setCenterOffset(x, y);
		dropoffController.setCenterOffset(x, y);
		  pickupController.setCenterOffset(x, y);
		  obstacleController.setCenterOffset(x, y);
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
			cout << "initial hashmap x,y = " << x << ", " << y << endl;
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

	  int step = 1;
	  Swarmie InitialRotate() {
		//Rotate to starting position...
		  float ninetyRotate = currTheta;
		  //float step2X;
		  //float step2Y;
		  cout << "Current theta is: " << currTheta << endl;

		  if (step == 1) {
			spiralSearchController.updateData(currX, currY, currTheta);
			cout << "step 1: rotating 90 degrees left..." << endl;

			//geometry_msgs::Point tempLocal;

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
					swarmie.left = 0.0;
					swarmie.right = 0.0;
					//rotateBool = false;
					//hardcodedPop = true;
					//initialMove = true;
					//step = 1;
					initialPosX = currX;
					initialPosY = currY;

					cout << "done rotating: step 1" << endl;
					startingTheta = currTheta;
					step = 2;
				}
				else {
					//sendDriveCommand(-30.0, 30.0);
					swarmie.left = -50.0;
					swarmie.right = 50.0;
					cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
				}



			}
			else
			{	//ninetyRotate = current theta
				//if the diff between the current theta and starting theta is >= 90 degrees, stop
				cout << "ninetyRotate = " << ninetyRotate << ", startingTheta = " << startingTheta << endl;
				if (abs(ninetyRotate - startingTheta) >= 1.5)
				{
					//sendDriveCommand(0.0, 0.0);
					swarmie.left = 0.0;
					swarmie.right = 0.0;
					//    rotateBool = false;
					//hardcodedPop = true;
					//initialMove = true;
					//GPSCenter = true;
					//step = 1;
					initialPosX = currX;
					initialPosY = currY;
					cout << "done rotating : step 1" << endl;
					startingTheta = currTheta;
					step = 2;

				 }	//else, turn right
				 else {
					//sendDriveCommand(-30.0, 30.0);
					swarmie.left = -50.0;
					swarmie.right = 50.0;
				 }

			}
		}
		
		else if (step == 2)
		{	
			spiralSearchController.updateData(currX, currY, currTheta);
			cout << "step 2: rotating 90 degrees left..." << endl;
			float turnSize = 1.5;
			bool exceedMag = false;

			ninetyRotate = currTheta;
			if (abs(startingTheta + turnSize) >= 3.142)
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
					swarmie.left = 0.0;
					swarmie.right = 0.0;
					cout << "done rotating: step 2" << endl;
					startingTheta = currTheta;
					//step2X = currX;
					//step2Y= currY;
					step = 3;
					//initialMove = false;
				}
				else {
					//sendDriveCommand(-30.0, 30.0);
					swarmie.left = -50.0;
					swarmie.right = 50.0;
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
				     cout << "done rotating: step 2" << endl;
				     startingTheta = currTheta;
				     //step2X = currX;
				     //step2Y = currY;
				     step = 3;
				      //initialMove = false;

			      }
			      else {
				    //sendDriveCommand(-30.0, 30.0);
				      swarmie.left = -50.0;
				      swarmie.right = 50.0;
			      }
			}
			
		}
		else if (step == 3)
		{
			//NEW BLOK FOR INITAL POP IDEA
			
			spiralSearchController.updateData(currX, currY, currTheta);
			cout << "Moving into place to begin spiral search..." << endl;
			//sendDriveCommand(30.0, 30.0);
			swarmie.left = 50.0;
			swarmie.right = 50.0;

			
			startingTheta = currTheta;
			//visitedLocationsPublisher.publish(initialPopf);
			//cout << "the point: " << initialPopf.data[0] << ", " << initialPopf.data[1] << " has been inserted/published..." << endl;
			
			//float displacement = sqrt(((currX - Step2X)*(currY - Position6X)) + ((currY - Position6Y)*(currY - Position6Y)));
			/*float displacement = calcDistance(currX, currY, step2X, step2Y);
			
			if (displacement >= 0.55)
			{	//???
				
				//step = 12;
				//this is temporary
				//Wheels.left = 0.0;
				//Wheels.right = 0.0;
				startingTheta = currTheta;

			}
			*/
		}
		
		return swarmie;
	  }
	
	void addVisitedLocation(float x, float y) {
        	if (currState == SPIRAL_SEARCH || (currState == AVOID_OBSTACLE && prevState == SPIRAL_SEARCH) ) 
		{
           	 	visitedLocations[normalizedValue(x)].insert(normalizedValue(y));
			//cout << "x,y: " << normalizedValue(x) << ", " << normalizedValue(y) << " has been added" << endl;
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
				cout << "done rotating: RIGHT 90" << endl;
				//step = 13;
				//initialMove = false;
				//mapTesting = true;
			}
			else {
				//sendDriveCommand(30.0, -30.0);
				swarmie.left = 50.0;
				swarmie.right = -50.0;
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
			    cout << "done rotating: RIGHT 90" << endl;
			    //step = 13;
			     // initialMove = false;
			     // mapTesting = true;
		      }
		      else {
			    //sendDriveCommand(30.0, -30.0);
			    swarmie.left = 50;
			    swarmie.right = -50.0;
		      }

		}
		
		return swarmie;
	}
};

#endif // LOGICCONTROLLER_H
