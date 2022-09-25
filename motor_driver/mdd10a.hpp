#ifndef MDD10A_HPP
#define MDD10A_HPP
#include "mbed.h"
#include "motor_driver.hpp"

class MDD10A : public MotorDriver
{
    public:
    MDD10A(PinName dir, PinName pwm);
    void Rotate(int dir, double pwm);
    void Rotate(double pwm) override;
    void Brake() override;
    DigitalOut dir_;
    PwmOut pwm_;
};

#endif