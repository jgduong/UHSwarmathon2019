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
	bool approachCube2 = false;
	bool reverse = false;
	bool halfStep = false;
	bool firstStep = false;
	bool checkInitialDist = true;
    Swarmie swarmie;
    float minX;
	float minZ;
    float indexOfClosestTag;
    int pickUpDelay = 0;
	int reverseDelay = 0;
    
    
    //void updateTags(float x, float y, float z) {
    void updateTags(vector<Tag> tagsReceived) {
	minX = tagsReceived[0].getPositionX();
	minZ = tagsReceived[0].getPositionZ();
	for (int i = 0; i < tagsReceived.size(); i++)
	{
		tags.push_back(tagsReceived[i]);
		if (tagsReceived[i].getPositionX() < minX)
		if ( abs( -0.03 - tagsReceived[i].getPositionX() ) < minX )
		{
			minX = tagsReceived[i].getPositionX();
			minZ = tagsReceived[i].getPositionZ();
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
	    cout << "detectionTimeout = " << detectionTimeout << endl;
          cout << "Target detected : in PICKUP state, minX is: " << minX << endl;
          cout << "index of centermost tag: " << indexOfClosestTag << ", x,z: " << tags[indexOfClosestTag].getPositionX() << ", " << tags[indexOfClosestTag].getPositionZ() << endl;
	  swarmie.pickupSuccess = false;
	    swarmie.dropoffSuccess = false;
	    
	    if (checkInitialDist ) {
		    cout << "minZ = " << minZ << endl;
		    if (minZ < 0.5) {
			    cout << "target is too close for pickup, backing up" << endl;
			    swarmie.left = -30.0;
			    swarmie.right = -30.0;
		    }
		    else {
			    cout << "zDistance is at an appropriate amount for pickup" << endl;    
			    swarmie.left = 0.0;
			    swarmie.right = 0.0;
			    checkInitialDist = false;
			    firstStep = true;
		    }
		    
	    }
        
            //center on cube
            if ( minX > -0.02 && detectionTimeout < 100 && !approachCube && firstStep)
            {
                //sendDriveCommand(6.0, -5.0);
                swarmie.left = 6.0;
                swarmie.right = -5.0;
            }
            else if ( minX < -0.04 & detectionTimeout < 100 && !approachCube && firstStep)
            {
                //sendDriveCommand(-5.0, 7.0);
                swarmie.left = -5.0;
                swarmie.right = 6.0;
            }
            else if (minX <= -0.02 && minX >= -0.04 && !approachCube && firstStep)
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
		    firstStep = false;
		detectionTimeout = 0;
		    
		    swarmie.wrist = 1.25;
		 swarmie.finger = M_PI_2 + 0.2;
            }
            else if (detectionTimeout >= 100)
            {
                //timeout
                //sendDriveCommand(5.0, 5.0);
                swarmie.left = 5.0;
                swarmie.right = 5.0;
                //aprilTagAcquireSequence = false;
                //mapTesting = true;
		detectionTimeout = 0;
		    cout << "pickUpController timeout reached" << endl;
		    swarmie.finger = 0;
		swarmie.wrist = 0;
            }
        
        if (approachCube) {
	    detectionTimeout++;
            cout << "Approaching cube..." << endl;
            cout << "z DistanceToCube is "  << zDistanceToCube << endl;
            distTravelled = sqrt( (selfX - startingX)*(selfX - startingX) + (selfY - startingY)*(selfY - startingY) );
            cout << "Distance travelled is : " << distTravelled << endl;
            if (distTravelled < (zDistanceToCube / 3.0) && !halfStep) {
                swarmie.left = 50.0;
                swarmie.right = 50.0;
            }
            else {
		    halfStep = true;
		    cout << "SECOND STEP of recalibration on cube... " << endl;
		    swarmie.left = 0.0;
		    swarmie.right = 0.0;
		 swarmie.wrist = 1.25;
		 swarmie.finger = M_PI_2 + 0.2;
		 if ( minX > -0.02 )
		    {
			//sendDriveCommand(6.0, -5.0);
			swarmie.left = 5.0;
			swarmie.right = -6.0;
			 cout << "calibrating by rotating right... " << endl;
		    }
		    else if ( minX < -0.025 )
		    {
			//sendDriveCommand(-5.0, 7.0);
			swarmie.left = -6.0;
			swarmie.right = 5.0;
			    cout << "calibrating by rotating left... " << endl;
		    }   
		    else if (minX <= -0.020 && minX >= -0.025)
		    {
			    approachCube = false;
			    approachCube2 = true;
			    detectionTimeout = 0;
			    startingX = selfX;
			    startingY = selfY;
			    distTravelled = 0.0;
			    
			    zDistanceToCube = minZ;
			    cout << "centered/calibrated a second time, new zDistanceToCube is: " << zDistanceToCube << endl;
		    }
            }
		
	    if (detectionTimeout >= 100)
	    {
		    swarmie.left = 5.0;
                    swarmie.right = 5.0;
		    detectionTimeout = 0;
		    cout << "pickUpController timeout reached" << endl;
		    swarmie.finger = 0;
		    swarmie.wrist = 0;
	    }
            
        }
	if (approachCube2)
	{
		detectionTimeout++;
		cout << "SECOND step of approaching cube... " << endl;
		cout << "Z distance to cube is: " << zDistanceToCube << endl;
		distTravelled = sqrt( (selfX - startingX)*(selfX - startingX) + (selfY - startingY)*(selfY - startingY) );
		cout << "distance travelled is: " << distTravelled << endl;
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
				approachCube2 = false;
				detectionTimeout = 0.0;
				reverse = true;
				reverseDelay = 0;
				distTravelled = 0.0;
			}
		}
		if (detectionTimeout >= 100)
		    {
			    detectionTimeout = 0;
			    cout << "pickUpController timeout reached" << endl;
				swarmie.finger = 0;
				swarmie.wrist = 0;
		    }
	}
	if (reverse)
	{
		swarmie.wrist = 0;
		swarmie.finger = 0;
		reverseDelay++;
		swarmie.left = -75.0;
                swarmie.right = -75.0;
		if (reverseDelay > 8)
		{
			reverseDelay = 0;
			reverse = false;
			swarmie.pickupSuccess = true;
			checkInitialDist = true;
			firstStep = false;
			halfStep = false;
		}
	}

          return swarmie;
    }


};
