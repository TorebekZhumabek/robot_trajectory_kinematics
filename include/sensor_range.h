#ifndef RANGESENSOR_H
#define RANGESENSOR_H


#include <sensor.h>
#include <iostream>

namespace arpro
{
class RangeSensor : public Sensor
{
public:
    RangeSensor(Robot &_robot, double _x, double _y, double _theta) : Sensor(_robot, _x,  _y,  _theta) {}

    void update(const Pose &_p)
    {
        // get all measurements to walls
        Pose p1, p2;
        double d;
        const double c = cos(_p.theta);
        const double s = sin(_p.theta);
        s_ = 100;
        for(int i=0;i<envir_->walls.size();++i)
        {
            p1 = envir_->walls[i];
            p2 = envir_->walls[(i+1)%envir_->walls.size()];

            d = (p1.x*p2.y - p1.x*_p.y - p2.x*p1.y + p2.x*_p.y + _p.x*p1.y-_p.x*p2.y)/
                    (p1.x*s - p2.x*s - p1.y*c + p2.y*c);
            if(d>0 && d<s_)
                s_ = d;
        }
        std::cout << "Measurement: " << s_ << std::endl;
    }

    void checkTwist(Twist &_t)
    {
        // we do not want to have a too large vx
        const double gain = .1;
        const double s_min = .5;
        const double s_act = 10;
        if(s_ < s_act)
        {
            if(_t.vx > gain*(s_-s_min))
            {
                std::cout << "changing " << _t.vx << " to " << gain*(s_-s_min) << std::endl;
                _t.vx = gain*(s_-s_min);
            }
        }
    }
};



}
#endif
