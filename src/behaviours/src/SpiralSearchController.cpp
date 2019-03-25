#include "SpiralSearchController.h"

/*
SpiralSearchController::SpiralSearchController() {

  fastVelPID.SetConfiguration(fastVelConfig());
  fastYawPID.SetConfiguration(fastYawConfig());

  slowVelPID.SetConfiguration(slowVelConfig());
  slowYawPID.SetConfiguration(slowYawConfig());

  constVelPID.SetConfiguration(constVelConfig());
  constYawPID.SetConfiguration(constYawConfig());

}
*/

SpiralSearchController::~SpiralSearchController() {}

void SpiralSearchController::Reset()
{
  waypoints.clear();

  if (stateMachineState == STATE_MACHINE_ROTATE)
  {
    stateMachineState = STATE_MACHINE_WAYPOINTS;
  }
}

Result DriveController::DoWork() {

  if(result.type == behavior)
  {
    if(result.b == noChange)
    {
      //if drive controller gets a no change command it is allowed to continue its previous action
      //normally this will be to follow waypoints but it is not specified as such.
    }

    else if(result.b == wait)
    {
      //do nothing till told otherwise
      left = 0.0;
      right = 0.0;
      stateMachineState = STATE_MACHINE_WAITING;
    }
    
    else if(result.type == precisionDriving)
    {

      //interpret input result as a precision driving command
      stateMachineState = STATE_MACHINE_PRECISION_DRIVING;

    }
    
    else if(result.type == waypoint)
    {
      //interpret input result as new waypoints to add into the queue
      ProcessData();

    }
    

switch(stateMachineState)
  {

  //Handlers and the final state of STATE_MACHINE are the only parts allowed to call INTERUPT
  //This should be d one as little as possible. I suggest using timeouts to set control bools to false.
  //Then only call INTERUPT if bool switches to true.
  case STATE_MACHINE_PRECISION_DRIVING:
  {
    cout << "Currently in PrecisionDriving state" << endl;
    ProcessData();
    break;
  }


  case STATE_MACHINE_WAYPOINTS:
  {
    cout << "Currently in Waypoint state" << endl;
    //Handles route planning and navigation as well as making sure all waypoints are valid.
    ProcessData();
    break;
  }

  case STATE_MACHINE_ROTATE:
  {
      cout << "Currently in Rotate state" << endl;
      break;
  }

}
  result.pd.right = right;
  result.pd.left = left;
  return result;

}



bool SpiralSearchController::ShouldInterrupt()
{
  if (interupt)
  {
    interupt = false;
    return true;
  }
  else
  {
    return false;
  }
}

void SpiralSearchController::ProcessData()
{
  //determine if the drive commands are waypoint or precision driving
  if (result.type == waypoint) {
    
    //sets logic controller into stand by mode while drive controller works
    cout << "Processing data for waypoint behavior" << endl;
  }
  else if (result.type == precisionDriving)
  {
    cout << "Processing data for PrecisionDriving behavior" << endl;
    //calculate inputs into the PIDS for precision driving

  }
}
  
void SetCurrentLocation(Point currentLocation) {
    this->currentLocation = currentLocation;
}

void SetVelocityData(float linearVelocity, float angularVelocity){
  this->linearVelocity = linearVelocity;
  this->angularVelocity = angularVelocity;
}
  
void SetAprilTags(vector<Tag> tags) {
  tag_boundary_seen = false;
  collection_zone_seen = false;
  count_left_collection_zone_tags = 0;
  count_right_collection_zone_tags = 0;

  // give Boundary tag type precedence, even if holding a target
  for (int i = 0; i < tags.size(); i++) {
    if (tags[i].getID() == 1) {
      tag_boundary_seen = true;
      timeSinceTags = current_time;
      return; // we don't check anything else if we're at a boundary
    }
  }

  // this loop is to get the number of center tags
  if (!targetHeld) {
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].getID() == 256) {
        collection_zone_seen = checkForCollectionZoneTags( tags[i] );
        timeSinceTags = current_time;
      }
    }
}  
}
/*  
void SetSonarData(float left, float center, float right) {
  left = sonarleft;
  right = sonarright;
  center = sonarcenter;
  ProcessData();
}
*/
void SetCenterLocationO(Point centerLocationOdom) {
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
}
  
void SetCurrentTimeInMilliSecs( long int time ) {
 current_time = time; 
}
