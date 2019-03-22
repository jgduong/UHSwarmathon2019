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
  
    Swarmie DoWork() {
      cout << "leftSonar is: " << leftSonar << endl;
      cout << "rightSonar is: " << rightSonar << endl;
      cout << "centerSonar is: " << centerSonar << endl;
      
      if (init) {
        swarmie.avoidObstacleSuccess = false;
        swarmie.pickupSuccess = false;
        swarmie.dropoffSuccess = false;
        init = false;
      }
      
      if (leftSonar <= 0.5 || centerSonar <= 0.5) {
        cout << "Obstacle on the left or center!" << endl;
        //swerve right
        swarmie.left = 50.0;
        swarmie.right = 20.0;
      }
      else if (rightSonar <= 0.5) {
        //swerve left
        cout << "Obstacle on the right!" << endl;
        swarmie.right = 50.0;
        swarmie.left = 20.0;
      }
      else {
        //no obstacle anymore
        cout << "Obstacle avoided" << endl;
        swarmie.left = 0.0;
        swarmie.right = 0.0;
        swarmie.avoidObstacleSuccess = true;
      }
      
      return swarmie;
    }


};
