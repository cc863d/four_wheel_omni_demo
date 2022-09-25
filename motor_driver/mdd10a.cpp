#include "mdd10a.hpp"

MDD10A::MDD10A(PinName dir, PinName pwm)
    : dir_(dir), pwm_(pwm)
{
    pwm_.period_us(50); // Cytron 10A MD max freq: 20kHz = 1 / 0.05 ms = 1 / 50 us
}

void MDD10A::Rotate(int dir, double pwm)
{
    dir_.write(dir);
    pwm_.write(pwm);
}

void MDD10A::Rotate(double pwm)
{
    int dir;
    if (pwm >= 0)
        dir = rotation_;
    if (pwm < 0)
        dir = !rotation_;
    pwm = fabs(pwm);

    dir_.write(dir);
    pwm_.write(pwm);
}

void MDD10A::Brake()
{
    dir_.write(0);
    pwm_.write(0);
}
