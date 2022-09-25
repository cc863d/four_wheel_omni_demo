#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP
class MotorDriver
{
    public:
    virtual void Rotate(double rate) = 0;
    virtual void Brake() = 0;
    int rotation_ = 0;
    void ChangeRotation() {
        rotation_ = !rotation_;
    }
};
#endif