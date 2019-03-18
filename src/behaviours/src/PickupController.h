#include "Wheels.h"
#include "Tag.h"
#include <iostream>
using namespace std;

class PickupController {

private:
    float detectionTimeout;
    float zDistanceToCube;
    bool approachTag;
    float tagX;
    float tagY;
    float tagZ;

public: 

    wheels Wheels;
    
    PickUpController() {
        detectionTimeout = 0.0;   
        zDistanceToCube = 0.0;
        approachTag = false;
    }
    
    void updateTags(float x, float y, float z) {
        tagX = x;
        tagY = y;
        tagZ = z;
    }
    
    wheels DoWork() {
          detectionTimeout++;
          cout << "Target detected : in PICKUP state" << endl;
          cout << "x, y, z of aprilTag: " << x << ", " << y << ", " << z << endl;
          //Wheels.left = 0.0;
          //Wheels.right = 0.0;
        
            //center on cube
            if ( tagX > 0 && detectionTimeOut < 200)
            {
                //sendDriveCommand(6.0, -5.0);
                Wheels.left = 6.0;
                Wheels.right = -5.0;
            }
            else if ( tagX < -0.002 && detectionTimeOut < 200)
            {
                //sendDriveCommand(-5.0, 7.0);
                Wheels.left = -5.0;
                Wheels.right = 6.0;
            }
            else if (x <= 0 && x >= -0.002)
            {
                cout << "centered on cube" << endl;
                //sendDriveCommand(0.0, 0.0);
                Wheels.left = 0.0;
                Wheels.right = 0.0;
                approachTag = true;
                zDistanceToCube = z;
                //aprilTagDetected = false;
                tagPickupTimer = 0.0;
                //middleStep = false;
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

          return Wheels;
    }


};
