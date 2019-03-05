#include <iostream>
#include "LogicController.h"
using namespace std;

class SpiralSearchController {

  public: 
    float currX;
    float currY;
    float currTheta;
  
    float centerOffsetX;
    float centerOffsetY;
  
    void updateData(float x, float y, float theta) {
      currX = x;
      currY = y;
      currTheta = theta;
    }
  
    void setCenterOffset(float x, float y) {
      centerOffsetX = x;
      centerOffsetY = y;
    }
    
    void DoWork(float currX, float currY, float currTheta) {
      cout << "Spiral Search Controller is working" << endl;
      cout << "Current Location: (" << currX << ", " << currY << ")" << endl;
      cout << "Theta = " << currTheta << endl;
    }


};
