#include "wheel.hpp"

Wheel::Wheel(Encoder * renc)
{
    renc_ = renc;
    desired_rad_per_s_ = 0;
}

double Wheel::GetCurrentAngularVelocity(void) {
    return renc_->GetAngularVelocity();
}

double Wheel::GetDesiredAngularVelocity(void) {
    return desired_rad_per_s_;
}

void Wheel::SetDesiredAngularVelocity(double desired_angular_velocity) {
    desired_rad_per_s_ = desired_angular_velocity;
}