#ifndef SWARMIE_H
#define SWARMIE_H

struct Swarmie {
  float left;
  float right;
  float wrist;
  float finger;
  //bool initialized;
  bool pickupSuccess;
  bool dropoffSuccess;
  bool avoidObstacleSuccess;
};

#endif 
