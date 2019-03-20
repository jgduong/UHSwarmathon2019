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
	float homeTheta;
	float distanceToHome = 0.0;
	float distTravelled = 0.0;
		
	bool initCalc = true;
	bool spinHome = false;
	bool driveToHome = false;
	Swarmie swarmie;
  
  public:
  
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
		if (initCalc) {
			homeTheta = atan2((0 - (currY + centerOffsetY)),(0 - (currX + centerOffsetX)));
			initialTheta = currTheta;
			initCalc = false;
			spinHome = true;
			swarmie.left = 0.0;
			swarmie.right = 0.0;
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
				if (abs(currTheta - homeTheta) <= 0.05)
				{
					//done rotating to home
					//sendDriveCommand(0.0, 0.0);
					swarmie.left = 0.0;
					swarmie.right = 0.0;
					cout << "done rotating" << endl;
					spinHome = false;
					driveToHome = true;
					initialX = currX + centerOffsetX;
					initialY = currX + centerOffsetY;
					//dropOffTimer = 0.0;
					distanceToHome = calcDistance(currX + centerOffsetX, currY + centerOffsetY, 0, 0);
					distanceToHome -= 0.5;
					//return swarmie;
				}
				else {
					//sendDriveCommand(-50.0, 50.0);
					swarmie.left = -50.0;
					swarmie.right = 50.0;
					//return swarmie;
				}
			}
			else if ( (turnSize < 0.0 && turnSize > -3.142) || turnSize >= 3.142) // right
			{
				cout << "currTheta is " << currTheta << endl;
				cout << "homeTheta is " << homeTheta << endl;
				if (abs(currTheta - homeTheta) <= 0.05)
				{
					//sendDriveCommand(0.0, 0.0);
					swarmie.left = 0.0;
					swarmie.right = 0.0;
					cout << "done rotating" << endl;
					spinHome = false;
					driveToHome = true;
					initialX = currX + centerOffsetX;
					initialY = currY + centerOffsetY;
					//dropOffTimer = 0.0;
					distanceToHome = calcDistance(currX + centerOffsetX, currY + centerOffsetY, 0, 0);
					distanceToHome -= 0.5;
					//return swarmie;
				}
				else {
					cout << "spinning towards home" << endl;
					//sendDriveCommand(50.0, -50.0);
					swarmie.left = 50.0;
					swarmie.right = -50.0;
					//return swarmie;
				}
			}
		}
		  else if (driveToHome) {
			  
			  //sendDriveCommand(100.0, 100.0);
			  //swarmie.left = 100.0;
			  //swarmie.right = 100.0;
			 
			  if (distanceToHome - distTravelled <= 0.01)
			  {
				  cout << "Made it to home base...dropping off cube!" << endl;
	  			  //fngr.data = M_PI_2;
				  //fingerAnglePublish.publish(fngr);
				  swarmie.finger = M_PI_2;
				  //sendDriveCommand(0.0, 0.0);
				  swarmie.left = 0.0;
				  swarmie.right = 0.0;
				  /*
				  if (dropOffTimer >= 30.0)
				  {
				  	  fngr.data = 0;
					  fingerAnglePublish.publish(fngr);
					  driveToHome = false;
					  returnToSpiralSearch = true;
					  reverseFromBaseTimer = 0.0;
					  oneEightyRotate_a = false;
					  oneEightyRotate_b = false;
				  }
				  */
				  //dropOffTimer++;
	       		  }
			  else {
				  cout << "desired distance is: " << distanceToHome << endl;
				  distTravelled = calcDist(currX, currY, initialX, initialY);
			  	  cout << "distance travelled is: " << distTravelled << endl;
				  swarmie.left = 100.0;
			  	  swarmie.right = 100.0;
			  }

		  }
		return swarmie;
	  }

};
