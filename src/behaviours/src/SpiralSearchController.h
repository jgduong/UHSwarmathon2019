#include <iostream>
#include <unordered_map>
#include <set>
#include "Swarmie.h"
#include "Calculations.h"
//#include "LogicController.h"
using namespace std;

class SpiralSearchController {

  public: 
    float currX;
    float currY;
    float normalizedX;
    float normalizedY;
    float currTheta;
  
    float centerOffsetX;
    float centerOffsetY;
    Swarmie swarmie;
    //Wheels.left = 0.0;
    //Wheels.right = 0.0;
  
    void updateData(float x, float y, float theta) {
      currX = x;
      currY = y;
      currTheta = theta;
      
      normalizedX = normalizedValue(x);
      normalizedY = normalizedValue(y);
    }
  
    void setCenterOffset(float x, float y) {
      centerOffsetX = x;
      centerOffsetY = y;
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
  

    
    Swarmie DoWork(unordered_map<float, set<float>> &visitedLocations) {
      swarmie.centerX = centerOffsetX;
		  swarmie.centerY = centerOffsetY;
      /*
      cout << "test: current location (" << currX << ", " << currY << ")";
      if (isVisited(currX, currY, visitedLocations))
      {
        cout << "exists in the hashmap" << endl;
      }
      else cout << "does not exist in the hashmap" << endl;
      //cout << "Spiral Search Controller is working" << endl;
      //cout << "Current Location: (" << currX << ", " << currY << ")" << endl;
      //cout << "Theta = " << currTheta << endl;
      Wheels.left = 0.0;
      Wheels.right = 0.0;
      */
      swarmie.pickupSuccess = false;
      swarmie.dropoffSuccess = false;
      
      float newTheta = currTheta - 1.53;

      float leftDrive = 0.0;
      float rightDrive = 0.0;

      //NEWEST METHOD CALCULATING ERROR
      bool maxFrontError = false;
      float FrontError;
      int m = 0;
      float newX;
      float newY;
      //starts at 2 so that it doesnt check on top of itself; checks 2 meters in front of it
      for (m = 2; m <= 8; m++)
      {
        newX = normalizedValue(currX + m*0.25*cos(currTheta));
        newY = normalizedValue(currY + m*0.25*sin(currTheta));
        cout << "current x, y" << normalizedX << ", " << normalizedY  << endl;
        if (isVisited(normalizedValue(newX), normalizedValue(newY), visitedLocations))
        {
          cout << "Front vector ended at m = " << m << "after checking: " << newX << ", " << newY << endl;
          maxFrontError = false;
          break;
        }
        else if (m == 8){
          maxFrontError = true;
        }
      }
      if (!maxFrontError)
      {
        float xWall = normalizedValue(newX);
        float yWall = normalizedValue(newY);
        FrontError = sqrt((xWall - (currX))*(xWall - (currX)) + (yWall - (currY))*(yWall - (currY)));
      }
      else {
        FrontError = 2;
      }
      cout << "FrontError is: " << FrontError << endl;
      if (FrontError <= 0.25)
      {
        leftDrive = -100;
        rightDrive = 100;
      }
      else if (FrontError <= 0.5)
      {
        leftDrive = FrontError*100 - 125;
        //leftDrive = FrontError*200 - 150;
        rightDrive = 100;
      }
      else {
        //leftDrive = 100*FrontError - 100;
        leftDrive = 50*FrontError;
        rightDrive = 100;

      }


      bool maxRightError = false;
      float RightError;
      int n = 0;
      for (n = 1; n <= 4; n++)
      {
        newX = normalizedValue(currX+ n*0.25*cos(currTheta - 1.571));
        newY = normalizedValue(currY + n*0.25*sin(currTheta - 1.571));

        if (isVisited(normalizedValue(newX), normalizedValue(newY), visitedLocations))
        {
          cout << "Right vector ended at n = " << n << "after checking: " << normalizedValue(newX) << ", " << normalizedValue(newY) << endl;
          maxRightError = false;
          break;
        }
        else if (n == 4) {
          maxRightError = true;
          cout << "Right vector max distance after checking: " << normalizedValue(newX) << ", " << normalizedValue(newY) << endl;
        }
      }
      if (!maxRightError)
      {
        float xWall = normalizedValue(newX);
        float yWall = normalizedValue(newY);
        cout << "RightError calculated with currentLocation and xWall,yWall = " << xWall << ", " << yWall << endl;
        //RightError = sqrt((xWall - (normalizedX))*(xWall - (normalizedX)) + (yWall - (normalizedY))*(yWall - (normalizedY)));
        RightError = sqrt((xWall - (currX))*(xWall - (currX)) + (yWall - (currY))*(yWall - (currY)));
      }
      else {
        cout << "max RightError" << endl;
        RightError = 1;
      }
      //RightError = RightError - 0.25;
      cout << "RightError is: " << RightError << ", desired is 0.25" << endl;
      cout << "CurrentLocation X,Y: " << normalizedX << ", " << normalizedY << endl;
      RightError = RightError - 0.25;
      if (maxFrontError)
      {	//MAKE THIS LESS SENSITIVE TO RIGHTERROR (linearize it)
        if (RightError < 0)
        {
          leftDrive = 100 + RightError*400;
          //leftDrive = 100 + RightError*200;
          //leftDrive = leftDrive + RightError*200;
        }
        //THIS ENTIRE ELSE IF BLOCK IS (barely)TESTED
        else if (RightError <= 0.25) {
          //rightDrive = 100 - (100*RightError);
          //rightDrive = -50;
          //leftDrive = (200*RightError)+50;
          leftDrive = 25 + (RightError*300);
          rightDrive = 100;
        }
        else {
          rightDrive = 100 - (200*RightError);
          //rightDrive = rightDrive - (200*RightError);
        }
      }
      //NEW, MAY WANT TO REMOVE 
      else if (FrontError > 0.5)
      {
        //rightDrive = 100 - (200*RightError);
      }
      cout << "sending drive commands: " << leftDrive << ", " << rightDrive << endl;
      //sendDriveCommand(leftDrive, rightDrive);
      swarmie.left = leftDrive;
      swarmie.right = rightDrive;

      return swarmie;
    }


};
