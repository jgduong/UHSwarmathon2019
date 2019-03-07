#include <iostream>
#include <unordered_map>
#include <set>
#include "LogicController.h"
using namespace std;

class SpiralSearchController {

  public: 
    float currX;
    float currY;
    float currTheta;
  
    float centerOffsetX;
    float centerOffsetY;
    struct wheels Wheels;
  
    void updateData(float x, float y, float theta) {
      currX = x;
      currY = y;
      currTheta = theta;
    }
  
    void setCenterOffset(float x, float y) {
      centerOffsetX = x;
      centerOffsetY = y;
    }

    bool isVisited(float x, float y) {
    if (visitedLocations.find(x) != visitedLocations.end()) {
      //x location exists in hashmap, check y coordinate
      if (visitedLocations[x].find(y) != visitedLocations[x].end()) {
        //y location also exists, so this coordinate has been visited
        return true;
      }
      else {
          return false;	
        }
      }
      else {
        return false;
      }
    }
    
    struct wheels DoWork(hashmap &visitedLocations) {
      cout << "test: current location (" << currX << ", " << currY << ")";
      if (visitedLocations.find(currX) != visitedLocations.end() && visitedLocations[currX].find(currY) != visitedLocations[currX].end())
      {
        cout << "exists in the hashmap" << endl;
      }
      else cout << "does not exist in the hashmap" << endl;
      //cout << "Spiral Search Controller is working" << endl;
      //cout << "Current Location: (" << currX << ", " << currY << ")" << endl;
      //cout << "Theta = " << currTheta << endl;
      Wheels->left = 0.0;
      Wheels->right = 0.0;
      return Wheels;
    }


};
