#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "mbed.h"
#include "PID.h"
#include "Encoder.h"

class Wheel
{
public:
    Wheel(Encoder * renc);
    Encoder * renc_;
    double GetDesiredAngularVelocity(void);
    double GetCurrentAngularVelocity(void);
    void SetDesiredAngularVelocity(double desired_angular_velocity); // unit:[rad/s]
    double desired_rad_per_s_;
};
#endif