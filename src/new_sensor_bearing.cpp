#include <sensor.h>
#include <math.h>
#include <new_sensor_bearing.h>
#include <iostream>

using namespace arpro;
using namespace std;

    //constructor using the mother constructor Sensor
    BearingSensor::BearingSensor(Robot &_robot, double _x, double _y, double _theta) : Sensor::Sensor(_robot, _x,  _y,  _theta)
    {

    }


    void BearingSensor::update(const Pose &_p){

        double angle;
        for(auto other: envir_->robots_){
            if(other != robot_){
                //we compute the robot between the robot and the other robot
              //  angle=atan2(_p.y - other->pose().y , _p.x - other->pose().x) - _p.theta;
                angle=atan2( other->pose().y - _p.y, other->pose().x - _p.x) - _p.theta;
                break;
            }
        }


        // the information from the sensor is stocked in the s_ variable
        s_=angle;

        // we set the angle back to [-pi,pi]
        s_ = fmod(s_+M_PI, 2*M_PI) - M_PI;
    }

    // check twist in sensor frame
    void BearingSensor::checkTwist(Twist &_t){
        double g=5;
        _t.w+=(g*s_);
    }

