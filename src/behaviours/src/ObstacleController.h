#include "Swarmie.h"
#include <iostream>
using namespace std;


class ObstacleController {
  
  public:
    //default to high numbers (no obstacles)
    float leftSonar = 10.0;
    float rightSonar = 10.0;
    float centerSonar = 10.0;
    bool init = true;
    Swarmie swarmie;

  
    void updateSonar(float left, float center, float right) {
      leftSonar = left;
      rightSonar = right; 
      centerSonar = center;
    }
  
    void DoWork() {
      
      if (init) {
        swarmie.pickupSuccess = false;
        swarmie.dropoffSuccess = false;
      }
      
      if (leftSonar <= 0.5) {
        cout << "Obstacle on the left!" << endl;
        //swerve right
        swarmie.left = 100.0;
        swarmie.right = 60.0;
      }
      else if (rightSonar <= 0.5) {
        //swerve left
        cout << "Obstacle on the right!" << endl;
        swarmie.right = 100.0;
        swarmie.left = 60.0;
      }
      
      
    }


};
