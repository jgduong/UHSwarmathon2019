#include "Swarmie.h"
#include "Calculations.h"
#include <iostream>
using namespace std;


class ObstacleController {

  private:
	float currX;
	float currY;
	float currTheta;
	float centerOffsetX;
	float centerOffsetY;

	float initialX;
	float initialY;
	float initialTheta;
	float desiredTheta;
	float distanceToHome = 0.0;
	float distTravelled = 0.0;
  
	  float SonarLeft = 0.0;
	  float SonarCenter = 0.0;
	  float SonarRight = 0.0;
		
	
  int prevState;
	Swarmie swarmie;
  
  public:
	bool initCalc = true;
	bool spinHome = false;
	bool driveToHome = false;
	bool backOff = false;
	bool rotate180 = false;
	bool backToSpiral = false;
  
  
	int delayCounter = 0;
	
	  enum States{
	    INIT = 0,
	    SPIRAL_SEARCH,
	    AVOID_OBSTACLE,
	    PICKUP,
	    DROPOFF,
	    FIND_SPIRAL_EDGE
	  };
  
	  void updateData(float x, float y, float theta) {
	      currX = x;
	      currY = y;
	      currTheta = theta;
	  }
	
	    void updateSonar(float left, float center, float right) {
		SonarLeft = left;
		SonarCenter = center;
		SonarRight = right;
	    }

	  void setCenterOffset(float x, float y) {
	      centerOffsetX = x;
	      centerOffsetY = y;
	  }
  
      Swarmie DoWork(int prev, bool noForwards) {
      cout << "Currently in the AVOID OBSTACLE state" << endl;
      
      swarmie.obstacleSuccess = false;
      
      prevState = prev;
      if (prevState == INIT) {
	      swarmie.pickupSuccess = false;
     	      swarmie.dropoffSuccess = false;
	      if (SonarRight <= 0.25) {
		      cout << "Obstacle detected on the right in the INIT stage" << endl;
		      swarmie.left = -100.0;
		      swarmie.right = 100.0;
	      }
	      else if (SonarLeft <= 0.25) {
		      cout << "Obstacle detected on the left in the init stage" << endl;
		      swarmie.left = 100.0;
		      swarmie.right = -100.0;
	      }
      }
      else if (prevState == SPIRAL_SEARCH)
      {
	  swarmie.pickupSuccess = false;
     	  swarmie.dropoffSuccess = false;
          if (SonarCenter <= 1.5)
          {
 		cout << "center detection still, sonarCenter is: " << SonarCenter << endl;
              swarmie.left = -100.0;
              swarmie.right = 100.0;
          }
          else if (SonarRight <= 1.5)
          {
		  cout << "right detection still, sonarRight is: " << SonarRight << endl;
              swarmie.left = -100.0;
              swarmie.right = 100.0;
		  if (SonarCenter >= 1.5 && SonarRight >= 0.3)
		  {
			  swarmie.left = 75.0;
			  swarmie.right = 75.0;
		  }
          }
          else if (SonarLeft <= 1.5)
          {
		  cout << "left detection still, sonarLeft is: " << SonarLeft << endl;
              swarmie.left = -100.0;
              swarmie.right = 100.0;
		  if (SonarCenter >= 1.5 && SonarRight >= 0.3)
		  {
			  swarmie.left = 75.0;
			  swarmie.right = 75.0;
		  }
          }
          else {
              cout << "obstacle controller has successfully rotated away from obstacle" << endl;
		swarmie.left = 75.0;
		swarmie.right = 75.0;
		 delayCounter++;	  
		  if (delayCounter >= 10)
		  {
             	 	swarmie.obstacleSuccess = true;
			 delayCounter = 0;
		  }
          }
        
      }
	else if (prevState == PICKUP)
	{
		swarmie.pickupSuccess = false;
     		swarmie.dropoffSuccess = false;
		
		if (SonarCenter <= 0.5)
		{
			swarmie.left = -50.0;
			swarmie.right = -50.0;
		}
		else if (SonarLeft <= 0.3 && SonarCenter >= 0.5)
		{
			swarmie.left = 100.0;
			swarmie.right = -100.0;
			
		}
		else if (SonarRight <= 0.3 && SonarCenter >= 0.5)
		{
			swarmie.left = -100.0;
			swarmie.right = 100.0;
		}
		else {
			swarmie.left = -50.0;
			swarmie.right = -50.0;
			delayCounter++;	  
			 if (delayCounter >= 4)
			 {
				swarmie.obstacleSuccess = true;
				  delayCounter = 0;
			 }
		}
	}
	      
	else if (prevState == DROPOFF && !noForwards)
	{
		swarmie.pickupSuccess = false;
     		swarmie.dropoffSuccess = false;
		if (SonarCenter <= 1.5 && SonarLeft >= 2.9 && SonarRight >= 2.9)
		  {
			cout << "center detection still, sonarCenter is: " << SonarCenter << endl;
		      swarmie.left = -100.0;
		      swarmie.right = 100.0;
		  }
		  else if (SonarRight <= 1.5)
		  {
			  cout << "right detection still, sonarRight is: " << SonarRight << endl;
		      swarmie.left = -100.0;
		      swarmie.right = 100.0;
			  if (SonarCenter >= 1.5 && SonarRight >= 0.3)
			  {
				  swarmie.left = 75.0;
				  swarmie.right = 75.0;
			  }
		  }
		  else if (SonarLeft <= 1.5)
		  {
			  cout << "left detection still, sonarLeft is: " << SonarLeft << endl;
		      swarmie.left = 100.0;
		      swarmie.right = -100.0;
			  if (SonarCenter >= 1.5 && SonarLeft >= 0.3)
			  {
				  swarmie.left = 75.0;
				  swarmie.right = 75.0;
			  }
		  }
		  else {
		      cout << "obstacle controller has successfully rotated away from obstacle" << endl;
		      swarmie.left = 75.0;
			swarmie.right = 75.0;
			 delayCounter++;	  
			  if (delayCounter >= 10)
			  {
				swarmie.obstacleSuccess = true;
				  delayCounter = 0;
			  }

		  }
	}
	else if (prevState == DROPOFF && noForwards)
	{
		cout << "noForward motions is TRUE" << endl;
		swarmie.pickupSuccess = false;
     		swarmie.dropoffSuccess = false;
		if (SonarCenter <= 0.35 && SonarLeft >= 2.9 && SonarRight >= 2.9)
		{
			cout << "center detection still, sonarCenter is: " << SonarCenter << endl;
		      swarmie.left = -100.0;
		      swarmie.right = 80.0;
		}
		else if (SonarLeft <= 0.35)
		{
			  cout << "left detection still, sonarLeft is: " << SonarLeft << endl;
		      swarmie.left = 80.0;
		      swarmie.right = -100.0;
		}
		else if (SonarRight <= 0.35)
		{
			cout << "right detection still, sonarRight is: " << SonarRight << endl;
		      swarmie.left = -100.0;
		      swarmie.right = 80.0;
		}
		else {
			cout << "obstacle controller has successfully rotated away from obstacle" << endl;
			swarmie.left = -30.0;
			swarmie.right = -30.0;
			 delayCounter++;	  
			  if (delayCounter >= 8)
			  {
				swarmie.obstacleSuccess = true;
				  delayCounter = 0;
			  }
		}
	}


      return swarmie;
      }

};
