#include <sensor.h>
#include <new_sensor_range.h>
#include <iostream>

using namespace arpro;
using namespace std;

    //constructor using the mother constructor Sensor
    RangeSensor::RangeSensor(Robot &_robot, double _x, double _y, double _theta) : Sensor::Sensor(_robot, _x,  _y,  _theta)
    {

    }


    void RangeSensor::update(const Pose &_p){
        Pose p1 , p2;
        s_=1000;
        for(int i=0;i<envir_->walls.size();i++){
             p1 = envir_->walls[i];
             p2 = envir_->walls[(i+1)%envir_->walls.size()];


            double d=((p1.x*p2.y) - (p1.x*_p.y) - (p2.x*p1.y) + (p2.x*_p.y) + (_p.x*p1.y) - (_p.x*p2.y)) / ((p1.x*sin(_p.theta)) - (p2.x*sin(_p.theta)) - (p1.y*cos(_p.theta)) + (p2.y*cos(_p.theta)));

            if(d>0 && d<s_)
                s_=d;

        }
    }

    // check twist in sensor frame
    void RangeSensor::checkTwist(Twist &_t){
        double g=0.1 , sm=0.1;

        if(_t.vx > g*(s_-sm))
            _t.vx = g*(s_-sm);
    }
