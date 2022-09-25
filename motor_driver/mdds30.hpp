#ifndef MDDS30_HPP
#define MDDS30_HPP

#include "mbed.h"
#include "motor_driver.hpp"

/* Serial Simplified */
class MDDS30 : public MotorDriver
{
    public:
    MDDS30(UnbufferedSerial * md_serial, int num);
    void Rotate(int dir, double rate);
    void Rotate(double rate) override;
    void Brake() override;

    int num_; // 通し番号
    UnbufferedSerial * md_;

};

#endif