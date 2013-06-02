#include <actuators/lights/LightController.hpp>
#include <iostream>

using namespace std;

int main()
{
    LightController lights;

    char in = 'B';

    while(true)
    {
        if(in == 'B')
            lights.SetSafetyLightState(Blinking);
        if(in == 'S')
            lights.SetSafetyLightState(Solid);
        cin >> in;
    }

    return 0;
}
