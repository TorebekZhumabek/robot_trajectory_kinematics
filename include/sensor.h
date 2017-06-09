#ifndef SENSOR_H
#define SENSOR_H

#include <string>
#include <envir.h>
#include <robot.h>

namespace arpro
{

class Sensor
{
public:
    // basic constructor
    Sensor(Robot &_robot, double _x, double _y, double _theta)
    {
        s_ = 0;
        s_history_.clear();
        robot_ = &_robot;
        _robot.attach(this, _x, _y, _theta);
    }

    inline static void setEnvironment(Environment &_envir) {envir_ = &_envir;}

    // update from current sensor pose
    virtual void update(const Pose &_p) = 0;

    // check twist in sensor frame
    virtual void checkTwist(Twist &_t) {}

    // check twist in robot frame
    void checkRobotTwist(Twist &_t, const Pose &_p)
    {
        bool display_twist = false;

        if(display_twist)
            std::cout << " Checking new sensor" << std::endl;
        if(display_twist)
            std::cout << "     Base robot twist: " << _t << std::endl;
        // twist in sensor frame
        _t = _t.transformInverse(_p);
        if(display_twist)
            std::cout << "     Base sensor twist: " << _t << std::endl;

        // check twist in sensor frame
        checkTwist(_t);

        if(display_twist)
            std::cout << "     Corrected sensor twist: " << _t << std::endl;

        // back to robot frame
        _t = _t.transformDirect(_p);
        if(display_twist)
            std::cout << "     Corrected robot twist: " << _t << std::endl;
    }

    // read current measurement
    double read() {return s_;}


protected:
    // current measurement
    double s_;
    // measurement history
    std::vector<double> s_history_;
    static Environment* envir_;
    Robot* robot_;
};

}



#endif // SENSOR_H
