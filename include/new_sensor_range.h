#ifndef NEW_SENSOR_RANGE_H
#define NEW_SENSOR_RANGE_H

#include <sensor.h>
#include <iostream>

namespace arpro
{
class RangeSensor : public Sensor
{
public:
    RangeSensor(Robot &_robot, double _x, double _y, double _theta);

    void update(const Pose &_p);

    // check twist in sensor frame
    void checkTwist(Twist &_t);

};
}

#endif // NEW_SENSOR_RANGE_H
