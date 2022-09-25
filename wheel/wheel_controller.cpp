#include "wheel_controller.hpp"

WheelController::WheelController(Encoder *enc, MotorDriver *md, PID *pid,
                                 float control_period_ms)
    : enc_(enc), md_(md), pid_(pid), wheel_(Wheel(enc)),
      control_period_ms_(control_period_ms) {
  current_apply_voltage_rate_ = 0;
  InitializePID();
}

void WheelController::InitializePID() {
  pid_->setInterval(0.001 *
                    control_period_ms_);      // ???????????????????s???????????
  pid_->setInputLimits(-6 * M_PI, +6 * M_PI); // ????]???H?H
  pid_->setOutputLimits(-1.0, 1.0);
  pid_->setMode(1); // AUTO
  pid_->setBias(0.0);
}

void WheelController::SetDesiredValue(double desired_value) {
  wheel_.SetDesiredAngularVelocity(desired_value);
}

void WheelController::SetControlParam(double control_param[3]) {
  pid_->setTunings(control_param[0], control_param[1], control_param[2]);
}

void WheelController::ApplyVoltageRate(float rate) {
  this->current_apply_voltage_rate_ = rate;
  this->md_->Rotate(current_apply_voltage_rate_);
}

void WheelController::ExecuteOneCycle() {

  if (control_state_ == ControlState::ENABLE) {
    pid_->setProcessValue(wheel_.GetCurrentAngularVelocity());
    pid_->setSetPoint(wheel_.GetDesiredAngularVelocity());
    current_apply_voltage_rate_ = pid_->compute();
  }
  if (control_state_ == ControlState::DISABLE) {
    // apply voltage rate does not change.
  }
  this->ApplyVoltageRate(current_apply_voltage_rate_);
}

void WheelController::ChangeControl(ControlState control_state) {
  control_state_ = control_state;
}
