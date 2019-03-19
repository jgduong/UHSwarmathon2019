#include "Wheels.h"
#include "Tag.h"
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
    
    float startingX;
    float startingY;
    float distTravelled = 0;

public: 

    wheels Wheels;
    
    
    void updateTags(float x, float y, float z) {
        tagX = x;
        tagY = y;
        tagZ = z;
    }
    
    wheels DoWork(float selfX, float selfY) {
          detectionTimeout++;
          cout << "Target detected : in PICKUP state" << endl;
          cout << "x, y, z of aprilTag: " << tagX << ", " << tagY << ", " << tagZ << endl;
          //Wheels.left = 0.0;
          //Wheels.right = 0.0;
        
            //center on cube
            if ( tagX > 0 && detectionTimeout < 200)
            {
                //sendDriveCommand(6.0, -5.0);
                Wheels.left = 6.0;
                Wheels.right = -5.0;
            }
            else if ( tagX < -0.002 && detectionTimeout < 200)
            {
                //sendDriveCommand(-5.0, 7.0);
                Wheels.left = -5.0;
                Wheels.right = 6.0;
            }
            else if (tagX <= 0 && tagX >= -0.002)
            {
                cout << "centered on cube" << endl;
                //sendDriveCommand(0.0, 0.0);
                Wheels.left = 0.0;
                Wheels.right = 0.0;
                
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
                Wheels.left = 5.0;
                Wheels.right = 5.0;
                //aprilTagAcquireSequence = false;
                //mapTesting = true;
            }
        
        if (approachCube) {
            cout << "Approaching cube..." << endl;
            cout << "z DistanceToCube is "  << zDistanceToCube << endl;
            distTravelled = sqrt( (selfX - startingX)*(selfX - startingX) + (selfY - startingY)*(selfY - startingY) );
            cout << "Distance travelled is : " << distTravelled << endl;
            if (distTravelled < (zDistanceToCube + 0.1)) {
                Wheels.left = 30.0;
                Wheels.right = 30.0;
            }
            else {
                cout << "Currently next to cube" << endl;
                Wheels.left = 0.0;
                Wheels.right = 0.0;
            }
            
        }

          return Wheels;
    }


};
