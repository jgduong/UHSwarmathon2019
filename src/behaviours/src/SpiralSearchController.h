#include <iostream>
#include "LogicController.h"
using namespace std;

class SpiralSearchController {

  public: 
    
    void DoWork(struct Swarmie thisSwarmie) {
      cout << "Spiral Search Controller is working" << endl;
      cout << "Current Location: (" << thisSwarmie->currX << ", " << thisSwarmie->currY << ")" << endl;
      cout << "Theta = " << thisSwarmie->currTheta << endl;
    }


};
