

#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = NULL;


Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);

    // default sampling time: 1/100 s
    dt_ = .01;

    //init the wheels
    initWheels(0.3,0.07,10);
}

void Robot::initWheels(double b, double r, double wmax){
    b_=b;
    r_=r;
    wmax_=wmax;
    wheels_init_=1;
}

void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}



void Robot::rotateWheels(double _left, double _right)
{

    // computing the speed and velocity of the robot
    double v=r_*(_left+_right)/2;
    double w=r_*(_left-_right)/(2*b_);
    double theta=pose_.theta;
    moveXYT(v*cos(theta),v*sin(theta),w);


}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    // computing the velocity of the left and right wheel
    double wl=(_v+(b_*_omega))/r_;
    double wr=(_v-(b_*_omega))/r_;

    // testing if the limit is respected by computing the ratio between the wl or wr and wmax
    double ratio=max(wl,wr)/wmax_;

    //if the ratio is lower than 1 then the speed limit is respected and wl and wr doesn't have to change
    if(ratio<1){
        ratio=1;
    }else{
        cout<<"Caution! The wheel's speed limit is not respected"<<endl;
        // we have to reduce the speed of the wheels to stay in the limit using the ratio previously computed
    }

    wl/=ratio;
    wr/=ratio;

    // give the wheel's velocity to th_e rotateWheels function to make them rotate
    rotateWheels(wl,wr);

}




// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frametwist.
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    //If we init the wheels we will use a more realistic model (moveVW) otherwise we use moveXYT
    if(wheels_init_){
        // go through all sensors to see if the robot twist is fine with them
        for(auto & sensor: sensors_)
        {
            sensor.sensor->update(sensor.pose.transformDirect(pose_));
            sensor.sensor->checkRobotTwist(_twist, sensor.pose);
        }

        // uses v-omega motion with wheel limits
       moveVW(_twist.vx,20*_twist.vy + _twist.w);


    }else{
        // uses X-Y motion (perfect but impossible in practice)
        moveXYT(_twist.vx, _twist.vy,_twist.w);
    }

}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

