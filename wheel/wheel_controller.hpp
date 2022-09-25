#ifndef WHEEL_CONTROLLER_HPP
#define WHEEL_CONTROLLER_HPP

#include "../motor_driver/mdd10a.hpp"
#include "../motor_driver/mdds30.hpp"
#include "Encoder.h"
#include "PID.h"
#include "mbed.h"
#include "wheel.hpp"


class WheelController {
public:
  enum class ControlState {
    ENABLE,
    DISABLE,
  };

  WheelController(Encoder *enc, MotorDriver *md, PID *pid,
                  float control_period);
  void InitializePID();

  // double GetDesiredValue(void);
  void SetDesiredValue(double desired_value);
  void SetControlParam(double control_param[3]);

  void ExecuteOneCycle(void);

  void ApplyVoltageRate(float rate);
  void ChangeControl(ControlState control_state);
  ControlState control_state_ = ControlState::DISABLE;

  Encoder *enc_;
  MotorDriver *md_;

  PID *pid_;
  Wheel wheel_;
  float control_period_ms_;
  float current_apply_voltage_rate_;
};

#endif
