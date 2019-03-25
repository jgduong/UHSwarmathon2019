#include "Swarmie.h"
#include "Tag.h"
#include "Calculations.h"
#include <iostream>
using namespace std;

class PickupController {

private:
    
    //float tagX;
    //float tagY;
    //float tagZ;
    vector<Tag> tags;
    
    float selfX;
    float selfY;
    
    float startingX;
    float startingY;
    float distTravelled = 0;

public: 
	float detectionTimeout = 0.0;
    float zDistanceToCube = 0.0;
    bool approachCube = false;
	bool reverse = false;
    Swarmie swarmie;
    float minX;
    float indexOfClosestTag;
    int pickUpDelay = 0;
	int reverseDelay = 0;
    
    
    //void updateTags(float x, float y, float z) {
    void updateTags(vector<Tag> tagsReceived) {
	minX = tagsReceived[0].getPositionX();
	for (int i = 0; i < tagsReceived.size(); i++)
	{
		tags.push_back(tagsReceived[i]);
		if (tagsReceived[i].getPositionX() < minX)
		if ( abs( -0.03 - tagsReceived[i].getPositionX() ) < minX )
		{
			minX = tagsReceived[i].getPositionX();
			indexOfClosestTag = i;
		}
	}
    }
    
    void updateData(float x, float y) {
	    	selfX = x;
	    	selfY = y;
	  }
    
    Swarmie DoWork() {
          detectionTimeout++;
          cout << "Target detected : in PICKUP state, minX is: " << minX << endl;
          cout << "index of centermost tag: " << indexOfClosestTag << ", x,z: " << tags[indexOfClosestTag].getPositionX() << ", " << tags[indexOfClosestTag].getPositionZ() << endl;
	  swarmie.pickupSuccess = false;
	    swarmie.dropoffSuccess = false;
        
            //center on cube
            if ( minX > -0.02 && detectionTimeout < 100 && !approachCube)
            {
                //sendDriveCommand(6.0, -5.0);
                swarmie.left = 6.0;
                swarmie.right = -5.0;
            }
            else if ( minX < -0.04 & detectionTimeout < 100 && !approachCube)
            {
                //sendDriveCommand(-5.0, 7.0);
                swarmie.left = -5.0;
                swarmie.right = 6.0;
            }
            else if (minX <= -0.02 && minX >= -0.04 && !approachCube)
            {
                cout << "centered on cube" << endl;
                //sendDriveCommand(0.0, 0.0);
                swarmie.left = 0.01;
                swarmie.right = 0.01;
                swarmie.wrist = 1.25;
                //swarmie.finger = M_PI_2;
		swarmie.finger = M_PI_2 + 0.2;    
                
                zDistanceToCube = tags[indexOfClosestTag].getPositionZ();
                //aprilTagDetected = false;
                //tagPickupTimer = 0.0;
                //middleStep = false;
                startingX = selfX;
                startingY = selfY;
                approachCube = true;
            }
            else
            {
                //timeout
                //sendDriveCommand(5.0, 5.0);
                swarmie.left = 5.0;
                swarmie.right = 5.0;
                //aprilTagAcquireSequence = false;
                //mapTesting = true;
		    
            }
        
        if (approachCube) {
            cout << "Approaching cube..." << endl;
            cout << "z DistanceToCube is "  << zDistanceToCube << endl;
            distTravelled = sqrt( (selfX - startingX)*(selfX - startingX) + (selfY - startingY)*(selfY - startingY) );
            cout << "Distance travelled is : " << distTravelled << endl;
            if (distTravelled < (zDistanceToCube)) {
                swarmie.left = 50.0;
                swarmie.right = 50.0;
            }
            else {
		pickUpDelay++;
                cout << "Currently next to cube" << endl;
                swarmie.left = 0.0;
                swarmie.right = 0.0;
                swarmie.finger = 0;
		if (pickUpDelay >= 10)
		{
			swarmie.wrist = 0;
		}
		if (pickUpDelay >= 20)
		{
			//swarmie.wrist = 0;
			//swarmie.pickupSuccess = true;
			pickUpDelay = 0;
			approachCube = false;
			detectionTimeout = 0.0;
			reverse = true;
			reverseDelay = 0;
		}
		    
            }
		
            
        }
	if (reverse)
	{
		reverseDelay++;
		swarmie.left = -75.0;
                swarmie.right = -75.0;
		if (reverseDelay > 8)
		{
			reverseDelay = 0;
			reverse = false;
			swarmie.pickupSuccess = true;
		}
	}

          return swarmie;
    }


};
