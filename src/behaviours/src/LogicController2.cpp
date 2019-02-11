#include "LogicController2.h"

LogicController2::LogicController2() {

  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCESS_STATE_SEARCHING;

  ProcessData();

  control_queue = priority_queue<PrioritizedController>();

}

LogicController2::~LogicController2() {}

void LogicController2::Reset() {

  std::cout << "LogicController2.Reset()" << std::endl;
  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCESS_STATE_SEARCHING;

  ProcessData();

  control_queue = priority_queue<PrioritizedController2>();
}

Result LogicController2::DoWork()
{
  Result result;

  // First, a loop runs through all the controllers who have a priority of 0 or
  // above with the largest number being most important. A priority of less than
  // 0 is an ignored controller (we will use -1 as the standard for an ignored
  // controller). If any controller needs an interrupt, the logic state is
  // changed to interrupt
  for(PrioritizedController2 cntrlr : prioritizedControllers2)
  {
    if(cntrlr.controller->ShouldInterrupt() && cntrlr.priority >= 0)
    {
      logicState = LOGIC_STATE_INTERRUPT;
      // Do not break out of the for loop! All shouldInterupts may need calling
      // in order to properly pre-proccess data.
    }
  }

  switch(logicState) {

  // ***************************************************************************
  // BEGIN LOGIC_STATE_INTERUPT
  // ***************************************************************************

  // Enter this state when an interrupt has been thrown or there are no pending
  // control_queue.top().actions.
  case LOGIC_STATE_INTERRUPT: {
    // Reset the control queue
    control_queue = priority_queue<PrioritizedController2>();

    // Check what controllers have work to do.. Every controller, where
    // HasWork() == true, will be added to the priority queue.
    for(PrioritizedController2 cntrlr : prioritizedControllers2) {
      if(cntrlr.controller->HasWork()) {
        if (cntrlr.priority < 0) {
          continue;
        }
        else {
          control_queue.push(cntrlr);
        }
      }
    }

    // If no controlers have work, report this to ROS Adapter and do nothing.
    if(control_queue.empty()) {
      result.type = behavior;
      result.b = wait;
      break;
    }
    else {
      // Default result state if someone has work. This safe gaurds against
      // faulty result types
      result.b = noChange;
    }

    // Take the top member of the priority queue and run its do work function.
    result = control_queue.top().controller->DoWork();

    // Analyze the result that was returned and do state changes accordingly.
    // Behavior types are used to indicate behavior changes.
    if(result.type == behavior) {

      // Ask for an external reset so the state of the controller is preserved
      // until after it has returned a result and gotten a chance to communicate
      // with other controllers.
      if (result.reset) {
        controllerInterconnect(); // Allow controller to communicate state data before it is reset.
        control_queue.top().controller->Reset();
      }

      // Ask for the procces state to change to the next state or loop around to the begining.
      //  enum ProcessState {
      //    _FIRST = 0,
      //    PROCESS_STATE_SEARCHING = 0,
      //    PROCESS_STATE_TARGET_PICKEDUP,
      //    PROCESS_STATE_DROP_OFF,
      //    _LAST,
      //    PROCESS_STATE_MANUAL // robot is under manual control
      //  };
      if(result.b == nextProcess) {
        if (processState == _LAST - 1) {
          processState = _FIRST;
        }
        else {
          processState = (ProcessState)((int)processState + 1);
        }
      }
      // Ask for the procces state to change to the previouse state or loop around to the end.
      else if(result.b == prevProcess) {
        if (processState == _FIRST) {
          processState = (ProcessState)((int)_LAST - 1);
        }
        else {
          processState = (ProcessState)((int)processState - 1);
        }
      }

      // Update the priorites of the controllers based upon the new process state.
      if (result.b == nextProcess || result.b == prevProcess) {
        ProcessData();
        result.b = wait;
        spiralSearchController.Reset(); // It is assumed that the drive controller may
                                 // be in a bad state if interrupted, so reset it.
      }
      break;
    }

    // Precision driving result types are when a controller wants direct
    // command of the robots actuators. LogicController facilitates the command
    // pass through in the LOGIC_STATE_PRECISION_COMMAND switch case.
    else if(result.type == precisionDriving) {

      logicState = LOGIC_STATE_PRECISION_COMMAND;
      break;

    }

    // Waypoints are also a pass through facilitated command but with a slightly
    // diffrent overhead. They are handled in the LOGIC_STATE_WAITING switch case.
    else if(result.type == waypoint) {

      logicState = LOGIC_STATE_WAITING;
      spiralSearchController.SetResultData(result);
      // Fall through on purpose to "case LOGIC_STATE_WAITING:"
    }

  }
  // ***************************************************************************
  // END LOGIC_STATE_INTERUPT
  // ***************************************************************************

  // ***************************************************************************
  // BEGIN LOGIC_STATE_WAITING
  // ***************************************************************************

  // This case is primarly when logic controller is waiting for drive controller
  // to reach its last waypoint.
  case LOGIC_STATE_WAITING: {
    // Ask drive controller how to drive: specifically, return commands to be
    // passed to the ROS Adapter such as left and right wheel PWM values in the
    // result struct.
    result = spiralSearchController.DoWork();

    // When out of waypoints, the drive controller will throw an interrupt.
    // However, unlike other controllers, drive controller is not on the
    // priority queue so it must be checked here.
    if (result.type == behavior) {
      if(spiralSearchController.ShouldInterrupt()) {
        logicState = LOGIC_STATE_INTERRUPT;
      }
    }
    break;
  }
  // ***************************************************************************
  // END LOGIC_STATE_WAITING
  // ***************************************************************************

  // ***************************************************************************
  // BEGIN LOGIC_STATE_PRECISION_COMMAND
  // ***************************************************************************

    // Used for precision driving pass through.
  case LOGIC_STATE_PRECISION_COMMAND: {

    // Unlike waypoints, precision commands change every update tick, so we ask
    // the controller for new commands on every update tick.
    result = control_queue.top().controller->DoWork();

    // Pass the driving commands to the drive controller so it can interpret them.
    spiralSearchController.SetResultData(result);

    // The interpreted commands are turned into proper initial_spiral_offset
    // motor commands to be passed the ROS Adapter such as left and right wheel
    // PWM values in the result struct.
    result = spiralSearchController.DoWork();
    break;

  }
  // ***************************************************************************
  // END LOGIC_STATE_PRECISION_COMMAND
  // ***************************************************************************
}
// end switch statment *********************************************************

  // Allow the controllers to communicate data between each other,
  // depending on the processState.
  controllerInterconnect();

  // Give the ROSAdapter the final decision on how it should drive.
  return result;
}
