#include <iostream>
#include "LogicController.h"
using namespace std;

class SpiralSearchController {

  public: 
    
    void DoWork(float currX, float currY, float currTheta) {
      cout << "Spiral Search Controller is working" << endl;
      cout << "Current Location: (" << currX << ", " << currY << ")" << endl;
      cout << "Theta = " << currTheta << endl;
    }


};
