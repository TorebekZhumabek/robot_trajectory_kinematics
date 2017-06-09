#ifndef NEW_SENSOR_BEARING_H
#define NEW_SENSOR_BEARING_H

#include <sensor.h>
#include <iostream>

namespace arpro
{
class BearingSensor : public Sensor
{
public:
    BearingSensor(Robot &_robot, double _x, double _y, double _theta);

    void update(const Pose &_p);

    // check twist in sensor frame
    void checkTwist(Twist &_t);

};
}


#endif // NEW_SENSOR_BEARING_H
