#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

#ifndef LOGICCONTROLLER_H
#define LOGICCONTROLLER_H

#include "Controller.h"
#include <SpiralSearchController.h>
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
struct swarmie {
  float currX;
  float currY;
  float currTheta;
  float leftVel;
  float rightVel;
  //int action;
  
  swarmie(float x, float y, float theta) {
    currX = x;
    currY = y;
    currTheta = theta;
    leftVel = 0;
    rightVel = 0;
    //action = 0;
  }
};

class LogicController {
  private:
    SpiralSearchController spiralSearchController;
  
  public: 
    int prevState;
    int currState;
    swarmie *thisSwarmie;
    //priority_queue<Controller> ControllerQueue;
  
    LogicController(float initialX, float initialY, float initialTheta) {
       prevState = 0;
       currState = 0;
       swarmie *Swarmie = new swarmie(initialX, initialY, initialTheta);
       thisSwarmie = Swarmie;
    }
/*  
    void updateQueue(int state) {
      //add obstacle controller
       if (state == SPIRAL_SEARCH) {
         ControllerQueue.push((Controller*)(&SpiralSearchController), 5);
       }
      else if (state == PICKUP) {
        //add pickup controller
      }
      else if (state == DROPOFF) {
         //add dropoff controller
      }
      else if (state == FIND_SPIRAL_EDGE) {
        //ADD FIND_SPIRAL_EDGE CONTROLLER 
      }
    }
*/  
  void DoWork(int state) {
    /*
    Controller *currController;
    for (int i = ControllerQueue.size(); i > 0; i--) {
        currController = ControllerQueue.pop();
        currController.DoWork();
    }
    */
    if (state == SPIRAL_SEARCH) {
      spiralSearchController.DoWork();
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
  
  void updateInfo(float x, float y, float theta) {
    thisSwarmie.currX = x;
    thisSwarmie.currY = y;
    thisSwarmie.currTheta = theta;
  }
  
};

#endif // LOGICCONTROLLER_H
