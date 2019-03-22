#include "Swarmie.h"
#include "Calculations.h"
#include <iostream>
using namespace std;


class DropoffController {

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
	float distToSpiral = 0.0;
		
	/*bool initCalc = true;
	bool spinHome = false;
	bool driveToHome = false;
	bool backOff = false;
	bool rotate180 = false;
	bool backToSpiral = false;*/
	Swarmie swarmie;
  
  public:
	float compareTheta = 0.0;
	bool initCalc = true;
	bool spinHome = false;
	bool driveToHome = false;
	bool backOff = false;
	bool rotate180 = false;
	bool backToSpiral = false;
	float homeTheta = 0.0;
	
	float spiralX = 0.0;
	float spiralY = 0.0;
	 float turnSize = 0.0;
  	
	  void updateData(float x, float y, float theta) {
	      currX = x;
	      currY = y;
	      currTheta = theta;
	  }

	  void setCenterOffset(float x, float y) {
	      centerOffsetX = x;
	      centerOffsetY = y;
	  }
  
	  Swarmie DoWork() {
		cout << "Currently in the DROPOFF state" << endl;
		  swarmie.pickupSuccess = false;
		  swarmie.dropoffSuccess = false;
		if (initCalc) {
			homeTheta = atan2((0 - currY),(0 - currX));
			desiredTheta = homeTheta + M_PI;
			if (desiredTheta >= M_PI)
			{
				desiredTheta -= 2 * M_PI;
			}
			
			initialTheta = currTheta;
			initCalc = false;
			spinHome = true;
			swarmie.left = 0.0;
			swarmie.right = 0.0;
			spiralX = currX;
			spiralY = currY;
			cout << "spiralX, spiralY: " << spiralX << ", " << spiralY << endl;
			//return swarmie;
	  	}
		else if (spinHome) {
			cout << "initialThetaBeforeHome is: " << initialTheta << endl;
			float turnSize = homeTheta - initialTheta;
			cout << "turnSize here is: " << turnSize << endl;
			bool exceedMag = false;
			//ninetyRotate = currTheta;

			if ( (turnSize >= 0.0 && turnSize < 3.142) || turnSize < -3.142) // left
			{
				if (abs(currTheta - homeTheta) <= 0.02)
				{
					//done rotating to home
					//sendDriveCommand(0.0, 0.0);
					swarmie.left = 0.0;
					swarmie.right = 0.0;
					cout << "done rotating" << endl;
					spinHome = false;
					driveToHome = true;
					initialX = currX;
					initialY = currY;
					//dropOffTimer = 0.0;
					distanceToHome = calcDistance(currX, currY, 0, 0);
					distanceToHome -= 0.4;
					//return swarmie;
				}
				else {
					//sendDriveCommand(-50.0, 50.0);
					swarmie.left = -40.0;
					swarmie.right = 40.0;
					//return swarmie;
				}
			}
			else if ( (turnSize < 0.0 && turnSize > -3.142) || turnSize >= 3.142) // right
			{
				cout << "currTheta is " << currTheta << endl;
				cout << "desiredTheta is " << homeTheta << endl;
				if (abs(currTheta - homeTheta) <= 0.02)
				{
					//sendDriveCommand(0.0, 0.0);
					swarmie.left = 0.0;
					swarmie.right = 0.0;
					cout << "done rotating" << endl;
					spinHome = false;
					driveToHome = true;
					initialX = currX;
					initialY = currY;
					//dropOffTimer = 0.0;
					distanceToHome = calcDistance(currX, currY, 0, 0);
					distanceToHome -= 0.4;
					//return swarmie;
				}
				else {
					cout << "spinning towards home" << endl;
					//sendDriveCommand(50.0, -50.0);
					swarmie.left = 40.0;
					swarmie.right = -40.0;
					//return swarmie;
				}
			}
		}
		  else if (driveToHome) {
			  
			  //sendDriveCommand(100.0, 100.0);
			  //swarmie.left = 100.0;
			  //swarmie.right = 100.0;
			 distTravelled = calcDistance(currX, currY, initialX, initialY);
			  if (distanceToHome - distTravelled <= 0.01)
			  {
				  cout << "Made it to home base...dropping off cube!" << endl;
	  			  //fngr.data = M_PI_2;
				  //fingerAnglePublish.publish(fngr);
				  swarmie.finger = M_PI_2 + 0.2;
				  //sendDriveCommand(0.0, 0.0);
				  swarmie.left = 0.0;
				  swarmie.right = 0.0;
				  initialX = currX;
				  initialY = currY;
				  backOff = true;
				  driveToHome = false;
				  
	       		  }
			  else {
				  cout << "desired distance is: " << distanceToHome << endl;
				  //distTravelled = calcDistance(currX, currY, initialX, initialY);
			  	  cout << "distance travelled is: " << distTravelled << endl;
				  swarmie.left = 100.0;
			  	  swarmie.right = 100.0;
			  }

		  }
		  else if (backOff) {
			  distTravelled = calcDistance(currX, currY, initialX, initialY);
			  cout << "Distance travelled is: " << distTravelled << endl;
			  //swarmie.finger = 0;
			  if ( 1 - distTravelled <= 0.02) {
				  cout << "Successfully backed out of home base" << endl;
				  swarmie.left = 0.0;
				  swarmie.right = 0.0;
				  backOff = false;
				  rotate180 = true;
				  swarmie.finger = 0;
				  
				  desiredTheta = atan2((0 - currY),(0 - currX));
				  compareTheta = currTheta;
				  
				   float turnSize = desiredTheta - currTheta;
			  }
			  else {
				cout << "Backing out of home base" << endl;
				  swarmie.left = -100.0;
				  swarmie.right = -100.0;
			  }
		  }
		  else if (rotate180) {
			 /* cout << "desiredTheta is: " << desiredTheta << endl;
			  cout << "currTheta is: " << currTheta << endl;
			  if (abs(desiredTheta - currTheta) <= 0.03) {
				  cout << "Done driving off of home base" << endl;
				  swarmie.left = 0.0;
				  swarmie.right = 0.0;
				  rotate180 = false;
				  backToSpiral = true;
				  desiredTheta = currTheta;
			  }
			  else {
				  cout << "Rotating away from home base" << endl;
				  swarmie.left = -50.0;
				  swarmie.right = 50.0;
			  }*/
			  
			  cout << "desiredTheta is: " << desiredTheta << endl;
			  cout << "currTheta is: " << currTheta << endl;
			 
			  
			  if (compareTheta != currTheta)
			  {
				  desiredTheta = atan2((spiralY - currY),(spiralX - currX));
				  compareTheta = currTheta;
				  turnSize = desiredTheta - currTheta;
			  }
			  
			cout << "turnSize here is: " << turnSize << endl;
			bool exceedMag = false;
			//ninetyRotate = currTheta;

			if ( (turnSize >= 0.0 && turnSize < 3.142) || turnSize < -3.142) // left
			{
				if (abs(currTheta - desiredTheta) <= 0.02)
				{
					//done rotating to home
					//sendDriveCommand(0.0, 0.0);
					swarmie.left = 0.0;
					  swarmie.right = 0.0;
				  	rotate180 = false;
				  	backToSpiral = true;
				}
				else {
					//sendDriveCommand(-50.0, 50.0);
					swarmie.left = -40.0;
					swarmie.right = 40.0;
					//return swarmie;
				}
			}
			else if ( (turnSize < 0.0 && turnSize > -3.142) || turnSize >= 3.142) // right
			{
				if (abs(currTheta - desiredTheta) <= 0.02)
				{
					//sendDriveCommand(0.0, 0.0);
					swarmie.left = 0.0;
				 	 swarmie.right = 0.0;
				  	rotate180 = false;
					  backToSpiral = true;
				}
				else {
					cout << "spinning towards home" << endl;
					//sendDriveCommand(50.0, -50.0);
					swarmie.left = 40.0;
					swarmie.right = -40.0;
					//return swarmie;
				}
			}
			  
		  }
		  else if (backToSpiral) {
              		cout << "distanceToSpiral is: " << distToSpiral << endl;
              		cout << "distanceTravelled is: " << distTravelled << endl;
              		distTravelled = calcDistance(currX, currY, initialX, initialY);
			if (abs(currTheta - desiredTheta) > 0.03) {
				  cout << "spinning to get back to spiral again" << endl;
				  distToSpiral = calcDistance(currX, currY, spiralX, spiralY);
				  swarmie.left = -30.0;
				  swarmie.right = 30.0;
			}  
			 
              		if ( distTravelled - distToSpiral <= 0.01) {
              		    cout << "Successfully drove back to spiral edge" << endl;
              		    swarmie.left = 0.0;
              		    swarmie.right = 0.0;
               		   backToSpiral = false;
				swarmie.dropoffSuccess = true;
				initCalc = true;
			
				distanceToHome = 0.0;
				distTravelled = 0.0;
				distToSpiral = 0.0;
             		 }
             		 else {
               		   cout << "Driving back to spiral" << endl;
                	 swarmie.left = 100.0;
                	  swarmie.right = 100.0;
              		}
         	 }
		return swarmie;
	  }

};
