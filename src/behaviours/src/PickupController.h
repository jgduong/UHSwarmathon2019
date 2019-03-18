#include "Wheels.h"
#include <iostream>
using namespace std;

class PickupController {

private:

public: 

    wheels Wheels;
    
    wheels DoWork() {
      cout << "Target detected : in PICKUP state" << endl;
      Wheels.left = 0.0;
      Wheels.right = 0.0;
        
      return Wheels;
    }


};
