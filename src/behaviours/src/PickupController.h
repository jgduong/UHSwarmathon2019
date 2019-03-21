#include "Swarmie.h"
#include "Tag.h"
#include "Calculations.h"
#include <iostream>
using namespace std;

class PickupController {

private:
    float detectionTimeout = 0.0;
    float zDistanceToCube = 0.0;
    bool approachCube = false;
    float tagX;
    float tagY;
    float tagZ;
    
    float selfX;
    float selfY;
    
    float startingX;
    float startingY;
    float distTravelled = 0;
    int pickupDelay = 0;

public: 

        float minX;
	int indexOfClosestTag;
        Swarmie swarmie;
	vector<Tag> tags;
    
    
	void updateTags(vector<Tag> tagsReceived) {
	    tagZ = tags[indexOfClosestTag].getPositionZ();
	    minX = tagsReceived[0].getPositionX();
	    for (int i = 0; i < tagsReceived.size(); i++)
	    {
		tags.push_back(tagsReceived[i]);
		if (tagsReceived[i].getPositionX() < minX)
		if ( abs( -0.005 - tagsReceived[i].getPositionX() ) < minX )
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
	  swarmie.pickupSuccess = false;
	  swarmie.dropoffSuccess = false;
	    //swarmie.initialized = true;
          cout << "Target detected : in PICKUP state" << endl;
          cout << "x, y, z of aprilTag: " << tagX << ", " << tagY << ", " << tagZ << endl;
          //Wheels.left = 0.0;
          //Wheels.right = 0.0;
        
            //center on cube
            if ( minX > 0 && detectionTimeout < 200 && !approachCube)
            {
		    //swarmie.pickupSuccess = false;
                    //sendDriveCommand(6.0, -5.0);
		    cout << "centering on cube" << endl;
                    swarmie.left = 6.0;
                    swarmie.right = -5.0;
            }
            else if ( minX < -0.01 & detectionTimeout < 200 && !approachCube)
            {
                    //sendDriveCommand(-5.0, 7.0);
		    cout << "centering on cube" << endl;
		    //swarmie.pickupSuccess = false;
                    swarmie.left = -5.0;
                    swarmie.right = 6.0;
            }
            else if (minX <= 0 && minX >= -0.01 && !approachCube)
            {
                cout << "centered on cube" << endl;
                //sendDriveCommand(0.0, 0.0);
                swarmie.left = 0.01;
                swarmie.right = 0.01;
                swarmie.wrist = 1.25;
                swarmie.finger = M_PI_2;
                detectionTimeout = 0;
		    
                zDistanceToCube = tagZ;
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
		detectionTimeout = 0;
                //aprilTagAcquireSequence = false;
                //mapTesting = true;
            }
        
        if (approachCube) {
            cout << "Approaching cube..." << endl;
            cout << "z DistanceToCube is "  << zDistanceToCube << endl;
            distTravelled = sqrt( (selfX - startingX)*(selfX - startingX) + (selfY - startingY)*(selfY - startingY) );
            cout << "Distance travelled is : " << distTravelled << endl;
            if (distTravelled < (zDistanceToCube)) {
                    swarmie.left = 30.0;
                    swarmie.right = 30.0;
            }
            else {
		    pickupDelay++;
                    cout << "Currently next to cube" << endl;
                    swarmie.left = 0.0;
                    swarmie.right = 0.0;
		    swarmie.finger = 0;
		    if (pickupDelay >= 20) {
			    swarmie.wrist = 0;
			    //swarmie.finger = 0;
			    swarmie.pickupSuccess = true;
			    approachCube = false;
			    detectionTimeout = 0;
			    pickupDelay = 0;
			    distTravelled = 0.0;
		    }
            }
            
        }

          return swarmie;
    }


};
