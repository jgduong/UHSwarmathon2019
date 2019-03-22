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
		
	bool initCalc = true;
	bool spinHome = false;
	bool driveToHome = false;
	bool backOff = false;
	bool rotate180 = false;
	bool backToSpiral = false;
  
  int prevState;
	Swarmie swarmie;
  
  public:
  
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
  
	Swarmie DoWork(int prev) {
      cout << "Currently in the AVOID OBSTACLE state" << endl;
      
      swarmie.obstacleSuccess = false;
      
      prevState = prev;
      if (prevState == SPIRAL_SEARCH)
      {
	  swarmie.pickupSuccess = false;
     	  swarmie.dropoffSuccess = false;
          if (SonarCenter <= 2.8)
          {
 		cout << "center detection still, sonarCenter is: " << SonarCenter << endl;
              swarmie.left = -100.0;
              swarmie.right = 100.0;
          }
          else if (SonarRight <= 2.9)
          {
		  cout << "right detection still, sonarRight is: " << SonarRight << endl;
              swarmie.left = -100.0;
              swarmie.right = 100.0;
		  if (SonarCenter >= 2.0 && SonarRight >= 0.3)
		  {
			  swarmie.left = 75.0;
			  swarmie.right = 75.0;
		  }
          }
          else if (SonarLeft <= 2.9)
          {
		  cout << "left detection still, sonarLeft is: " << SonarLeft << endl;
              swarmie.left = -100.0;
              swarmie.right = 100.0;
		  if (SonarCenter >= 2.0 && SonarRight >= 0.3)
		  {
			  swarmie.left = 75.0;
			  swarmie.right = 75.0;
		  }
          }
          else {
              cout << "obstacle controller has successfully rotated away from obstacle" << endl;
              swarmie.obstacleSuccess = true;
            
          }
        
      }
	if (prevState == DROPOFF)
	{
		swarmie.pickupSuccess = false;
     		swarmie.dropoffSuccess = false;
		if (SonarCenter <= 2.8)
		  {
			cout << "center detection still, sonarCenter is: " << SonarCenter << endl;
		      swarmie.left = -100.0;
		      swarmie.right = 100.0;
		  }
		  else if (SonarRight <= 2.9)
		  {
			  cout << "right detection still, sonarRight is: " << SonarRight << endl;
		      swarmie.left = -100.0;
		      swarmie.right = 100.0;
			  if (SonarCenter >= 2.0 && SonarRight >= 0.3)
			  {
				  swarmie.left = 75.0;
				  swarmie.right = 75.0;
			  }
		  }
		  else if (SonarLeft <= 2.9)
		  {
			  cout << "left detection still, sonarLeft is: " << SonarLeft << endl;
		      swarmie.left = 100.0;
		      swarmie.right = -100.0;
			  if (SonarCenter >= 2.0 && SonarLeft >= 0.3)
			  {
				  swarmie.left = 75.0;
				  swarmie.right = 75.0;
			  }
		  }
		  else {
		      cout << "obstacle controller has successfully rotated away from obstacle" << endl;
		      swarmie.obstacleSuccess = true;

		  }
	}


      return swarmie;
      }

};
