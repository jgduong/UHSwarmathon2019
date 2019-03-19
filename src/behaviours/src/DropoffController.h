#include "Swarmie.h"
#include <iostream>
using namespace std;


class DropoffController {

  private:
  float currX;
  float currY;
  float currTheta;
  float centerOffsetX;
  float centerOffsetY;
  
  float initialTheta;
  float homeTheta;
  bool spinHome = true;
  Swarmie swarmie;
  
  public:
  
  void updateData(float x, float y, float x) {
      currX = x;
      currY = y;
      currTheta = theta;
  }
  
  void setCenterOffset(float x, float y) {
      centerOffsetX = x;
      centerOffsetY = y;
  }
  
  Swarmie DoWork() {
    if (spinHome) {
      homeTheta = atan2((0 - (currentLocationOdom.y + centerOffsetY)),(0 - (currentLocationOdom.x + centerOffsetX)));
      initialTheta = currTheta;
    }
		//returnToHome = false;
    //rotateToHome = true;
    
  
  }

};
