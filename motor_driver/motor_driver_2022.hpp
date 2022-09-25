#ifndef MOTORDRIVER2022_HPP
#define MOTORDRIVER2022_HPP

#include "mbed.h"
#include "motor_driver.hpp"
#include <algorithm>
#include <ratio>


class MotorDriver2022 : public MotorDriver
{
    public:
    MotorDriver2022(PinName pwmh, PinName pwml, PinName sr, PinName phase);
    void Rotate(double rate) override;
    void Brake() override;
    void Coast();

    enum class ModeRectification{
        DIODE,
        SYNCHRONOUS,
    };
    enum class ModeDecay{
        FAST,
        SLOW,
    };
    enum class ModeDrivingSide{
        NONE,
        HIGHSIDE,
        LOWSIDE
    };

    ModeRectification mode_rectification_;
    ModeDecay mode_decay_;
    ModeDrivingSide mode_driving_side_;
    
    /// 整流方式の切り替えを行う（同期/ダイオード（＝非同期））
    /// ディケイの設定によっては切り替えられないことがある（ディケイモードの優越）
    int SetModeRectification(ModeRectification mode_rectification);
    /// 減衰方法の設定（ファスト/スロー）
    int SetModeDecay(ModeDecay mode_decay);
    /// FET のどちらを駆動するか切り替える
    /// ディケイの設定によっては切り替えられないことがある（ディケイモードの優越）
    int SetModeDrivingSide(ModeDrivingSide mode_driving_side);

    PwmOut pwmh_;
    PwmOut pwml_;
    DigitalOut sr_;
    DigitalOut phase_;    

};


MotorDriver2022::MotorDriver2022(PinName pwmh, PinName pwml, PinName sr, PinName phase) : pwmh_(pwmh), pwml_(pwml), sr_(sr), phase_(phase)
{
    mode_rectification_ = ModeRectification::SYNCHRONOUS;
    mode_decay_ = ModeDecay::SLOW;
    mode_driving_side_ = ModeDrivingSide::LOWSIDE;
}

int MotorDriver2022::SetModeRectification(ModeRectification mode_rectification)
{
    if (mode_decay_ == ModeDecay::FAST) {
        sr_.write(0);
        mode_rectification_ = ModeRectification::DIODE;
        return 1;
    }

    mode_rectification_ = mode_rectification;
    if (mode_rectification_ == ModeRectification::SYNCHRONOUS) {
        sr_.write(1);
    } else {
        sr_.write(0);
    }
    return 0;
}

int MotorDriver2022::SetModeDecay(ModeDecay mode_decay) {
    mode_decay_ = mode_decay;
    if (mode_decay_ == ModeDecay::FAST) {
        mode_rectification_ = ModeRectification::DIODE;
        mode_driving_side_ = ModeDrivingSide::NONE;
    }
    return 0;
}

int MotorDriver2022::SetModeDrivingSide(ModeDrivingSide mode_driving_side) {

    mode_driving_side_ = mode_driving_side;
    if (mode_decay_ == ModeDecay::FAST) {
        mode_driving_side_ = ModeDrivingSide::NONE;
        return 1;
    }
    return 0;
}


void MotorDriver2022::Rotate(double rate){
    phase_.write((rate>0 ? !rotation_ : rotation_));

    if (mode_decay_ == ModeDecay::FAST) {
        pwml_.write(abs(rate));
        pwmh_.write(abs(rate));
    } else if (mode_driving_side_ == ModeDrivingSide::HIGHSIDE) {
        pwmh_.write(abs(rate));
        pwml_.write(1);
    } else if (mode_driving_side_ == ModeDrivingSide::LOWSIDE) {
        pwml_.write(abs(rate));
        pwmh_.write(1);
    }
}
void MotorDriver2022::Brake(){
    if (mode_decay_ == ModeDecay::FAST) {
        // !!!! can not brake in the fast decay mode !!!
        pwmh_.write(0);
        pwml_.write(0);
    } else if (mode_driving_side_ == ModeDrivingSide::HIGHSIDE) {
        pwmh_.write(0);
        pwml_.write(1);
    } else if (mode_driving_side_ == ModeDrivingSide::LOWSIDE) {
        pwml_.write(0);
        pwmh_.write(1);
    }
}
void MotorDriver2022::Coast(){
    pwmh_.write(0);
    pwml_.write(0);
}


#endif