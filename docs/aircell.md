#AirCell APIs
## Introduction
AirCell is the battery simulation framework under AirSim. You can use C/C++ API to query battery statistics for energy-awareness.

## Hello Drone With AirCell
Here's very quick overview of how to use AirCell APIs using C++:

```
#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

int main() 
{
    using namespace std;
    msr::airlib::MultirotorRpcLibClient client;

    cout << "Press Enter to enable API control" << endl; cin.get();
    client.enableApiControl(true);

    cout << "Press Enter to arm the drone" << endl; cin.get();
    client.armDisarm(true);

    cout << "Press Enter to takeoff" << endl; cin.get();
    client.takeoff(5);

    cout << "Press Enter to move 5 meters in x direction with 1 m/s velocity" << endl; cin.get();  
    auto position = client.getPosition(); // from current location
    client.moveToPosition(position.x() + 5, position.y(), position.z(), 1);

    cout << "Press Enter to land" << endl; cin.get();
    client.land();

    cout << "SoC(%) " << client.getStateOfCharge() << endl;
    cout << "Voltage " << client.getVoltage() << endl;

    return 0;
}

```

You can find a ready to run project in HelloDrone folder in the repository. See more about [Hello Drone](hello_drone.md).

## Implementation Details
There is a Google Slide (https://docs.google.com/presentation/d/1SlmCuaR_3ZNTfGPdnLAM4kYHgDA7GBYuTjLglaRhwsw/edit?usp=sharing) which explains the overview of AirCell. I just summarize the files I have modifed in order to implement AirCell:

1. *AirLib/include/common/CommonStructs.hpp*: Add a struct named BatteryInfo to expose battery-related information from physics engine.
2. *AirLib/include/physics/Battery.hpp b/AirLib/include/physics/Battery.hpp*: Implement battery simulation and power estimator.
3. *AirLib/include/physics/FastPhysicsEngine.hpp*: Instrument physics simulator loop to add battery simulation code.
4. *AirLib/include/physics/PhysicsBody.hpp*: Add a Battery pointer field and related methods in PhysicsBody class.
5. *AirLib/include/vehicles/multirotor/MultiRotor.hpp*: Creates Battery for MultiRoto class and propagate battery information to controller in update() method.
6. *AirLib/include/vehicles/multirotor/controllers/DroneControllerBase.hpp*: Add BatteryInfo API in drone DroneControllerBase class for interacting between physics simulator and other module.
7. *AirLib/include/vehicles/multirotor/api/MultirotorRpcLibClient.hpp*: Add RPC call stub for AirCell at client side.
8. *AirLib/src/vehicles/multirotor/api/MultirotorRpcLibServer.cpp*: Add RPC call stub for AirCell at server side.
9. *AirLib/src/vehicles/multirotor/controllers/DroneControllerBase.cpp*: Implement BatteryInfo related methods in DroneControllerBase class.
      
