#ifndef BEARINGSENSOR_H
#define BEARINGSENSOR_H


#include <sensor.h>
#include <iostream>

namespace arpro
{
class BearingSensor : public Sensor
{
public:
    BearingSensor(Robot &_robot, double _x, double _y, double _theta) : Sensor(_robot, _x, _y, _theta) {}


    void update(const Pose &_p)
    {
        // look for first other robot
        for(auto other: envir_->robots_)
            if(other != robot_)
            {
                s_ = atan2(other->pose().y - _p.y, other->pose().x - _p.x) - _p.theta;
                break;
            }
        s_ = fmod(s_+M_PI, 2*M_PI) - M_PI;
    }

    void checkTwist(Twist &_t)
   {
        _t.w += 5*s_;
    }
};



}
#endif
