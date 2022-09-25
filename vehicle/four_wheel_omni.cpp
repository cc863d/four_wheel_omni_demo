#include "four_wheel_omni.hpp"
#include <array>
FourWheelOmni::FourWheelOmni(MotorDriver *md[4]) {
  for (int i = 0; i < 4; i++) {
    md_[i] = md[i];
    wheel_controller_[i] = nullptr;
  }
  renc_odometry_xy_[0] = nullptr;
  renc_odometry_xy_[1] = nullptr;
  ahrs_ = nullptr;
}

int FourWheelOmni::AddWheelController(WheelController *wheel_controller[4]) {
  for (int i = 0; i < 4; i++) {
    wheel_controller_[i] = wheel_controller[i];
  }

  encoder_interrupter_thread_.start(
      callback(this, &FourWheelOmni::EncoderDrive));
  execute_one_cycle_interrupter_thread_.start(
      callback(this, &FourWheelOmni::ExecuteControlForOneCycle));

  encoder_interrupter_thread_.set_priority(osPriorityHigh);
  execute_one_cycle_interrupter_thread_.set_priority(osPriorityAboveNormal);

  encoder_interrupter_.attach(
      callback(this, &FourWheelOmni::EncoderInterrupterThreadDriver),
      (std::chrono::microseconds)(encoder_interrupt_time_us_));
  execute_one_cycle_interrupter_.attach(
      callback(this, &FourWheelOmni::ExecuteControlForOneCycleThreadDriver),
      (std::chrono::microseconds)control_cycle_us);

  return 0;
}

int FourWheelOmni::AddAhrs(BNO055 *ahrs) {
  if (ahrs_ != nullptr) {
    return 1;
  }
  ahrs_ = ahrs;
  ahrs_->reset();
  // Check that the BNO055 is connected and flash LED if not
  if (!ahrs_->check()) {
    while (true) {
      printf("NOOOOOOO!\n");
    }
  }
  ahrs_->setmode(OPERATION_MODE_NDOF);
  ahrs_->set_angle_units(RADIANS); // なぜかこれを設定しても deg になる
  ahrs_->set_accel_units(MPERSPERS);
  return 0;
}
int FourWheelOmni::AddEncoderForOdometry(Encoder *renc_odometry_xy[2]) {
  renc_odometry_xy_[0] = renc_odometry_xy[0];
  renc_odometry_xy_[1] = renc_odometry_xy[1];
  return 0;
}

void FourWheelOmni::EncoderDrive() {
  while (true) {
    ThisThread::flags_wait_all(1, true);
    if (renc_odometry_xy_[0] != nullptr) {
      // オドメトリ用エンコーダについて計測
      for (int i = 0; i < 2; i++) {
        renc_odometry_xy_[i]->Counter();
      }
    }

    for (int i = 0; i < 4; i++) {
      wheel_controller_[i]->enc_->Counter();
    }
  }
}

void FourWheelOmni::ExecuteControlForOneCycle() {
  while (true) {
    ThisThread::flags_wait_all(1, true);
    counter_ += 1;
    if (counter_ >= max_count_) {
      execute_one_cycle_interrupter_.detach();
    }
    double old_x = current_configuration_[0];
    double old_y = current_configuration_[1];

    //
    for (ConfigurationCorrector cc : configuration_correctors_) {
      bool is_all_conditions_ok = true;
      for (DigitalIn *c : cc.conditions_) {
        if (c->read() ==
            0) { // CHANGE 0 or 1 wheather pullup or pulldown of digitalin
          is_all_conditions_ok = false;
          break;
        }
      }
      if (is_all_conditions_ok) {
        *(cc.fixed_point_) = cc.value_;
      }
    }

    if (renc_odometry_xy_[0] != nullptr) {
      // When attached with wheels for odometry
      current_configuration_[0] = renc_odometry_xy_[0]->Distance();
      current_configuration_[1] = renc_odometry_xy_[1]->Distance();

      current_velocity_[0] = renc_odometry_xy_[0]->GetAngularVelocity() *
                             renc_odometry_xy_[0]->_omni_diameter;
      current_velocity_[1] = renc_odometry_xy_[1]->GetAngularVelocity() *
                             renc_odometry_xy_[1]->_omni_diameter;
    } else if (wheel_controller_[0] != nullptr) {
      // When NOT attached with wheels for odometry
      // => Odometry by drive wheels
      double delta_x = 0.5f * 0.7071f *
                       (+(wheel_controller_[0]->enc_->Distance() -
                          previous_wheel_distance[0]) +
                        (wheel_controller_[1]->enc_->Distance() -
                         previous_wheel_distance[1]) -
                        (wheel_controller_[2]->enc_->Distance() -
                         previous_wheel_distance[2]) -
                        (wheel_controller_[3]->enc_->Distance() -
                         previous_wheel_distance[3]));

      double delta_y =
          0.5f * ((-0.7071f * -(previous_wheel_distance[0] -
                                wheel_controller_[0]->enc_->Distance())) +
                  (+0.7071f * -(previous_wheel_distance[1] -
                                wheel_controller_[1]->enc_->Distance())) +
                  (+0.7071f * -(previous_wheel_distance[2] -
                                wheel_controller_[2]->enc_->Distance())) +
                  (-0.7071f * -(previous_wheel_distance[3] -
                                wheel_controller_[3]->enc_->Distance())));

      if (ahrs_ == nullptr) {
        current_configuration_[2] = ((wheel_controller_[0]->enc_->Distance() +
                                      wheel_controller_[1]->enc_->Distance() +
                                      wheel_controller_[2]->enc_->Distance() +
                                      wheel_controller_[3]->enc_->Distance())) /
                                    (4.0 * wheel_separation_width_) * 180.0 /
                                    M_PI; // To degree
      }

      current_configuration_[0] +=
          cos(current_configuration_[2] * M_PI / 180.0) * delta_x -
          sin(current_configuration_[2] * M_PI / 180.0) * delta_y;
      current_configuration_[1] +=
          sin(current_configuration_[2] * M_PI / 180.0) * delta_x +
          cos(current_configuration_[2] * M_PI / 180.0) * delta_y;
      //   current_configuration_[0] += delta_x;
      //   current_configuration_[1] += delta_y;

      current_velocity_[0] = delta_x * 100000 / control_cycle_us;
      current_velocity_[1] = delta_y * 100000 / control_cycle_us;

      for (int i = 0; i < 4; i++) {
        previous_wheel_distance[i] = wheel_controller_[i]->enc_->Distance();
      }
    }
    for (int i = 0; i < 4; i++) {
      wheel_controller_[i]->ExecuteOneCycle();
    }
  }
}
void FourWheelOmni::EncoderInterrupterThreadDriver() {
  encoder_interrupter_thread_.flags_set(1);
}
void FourWheelOmni::ExecuteControlForOneCycleThreadDriver() {
  execute_one_cycle_interrupter_thread_.flags_set(1);
}

void FourWheelOmni::SetRateVoltage(std::array<double, 4> rate) {
  // In order to match the function name and behaviour
  // The sign is always + and is not changed!
  for (int i = 0; i < 4; i++) {
    if (this->wheel_controller_[i] == nullptr) {
      this->md_[i]->Rotate(+rate[i]);
    } else {
      this->wheel_controller_[i]->ApplyVoltageRate(+rate[i]);
    }
  }
}

void FourWheelOmni::SetDesiredAngularVelocities(double angular_velocities[4]) {
  for (int i = 0; i < 4; i++) {
    wheel_controller_[i]->SetDesiredValue(angular_velocities[i]);
  }
}

void FourWheelOmni::SetDesiredVelocities(
    std::array<double, 3> desired_velocity_vehicle) {
  double set_velocity[3]; // 指示しようとしている動き x'[mm/s] , y'[mm/s],
                          // theta'[rad/s]
  double set_velocities[4];         // 各車輪の周速度
  double set_angular_velocities[4]; // 各車輪の角速度

  for (int i = 0; i < 3; i++) {
    set_velocity[i] = desired_velocity_vehicle[i];
  }

  set_velocities[0] = (set_velocity[0] - set_velocity[1] +
                       set_velocity[2] * wheel_separation_width_);
  set_velocities[1] = (set_velocity[0] + set_velocity[1] +
                       set_velocity[2] * wheel_separation_width_);
  set_velocities[2] = (-set_velocity[0] + set_velocity[1] +
                       set_velocity[2] * wheel_separation_width_);
  set_velocities[3] = (-set_velocity[0] - set_velocity[1] +
                       set_velocity[2] * wheel_separation_width_);

  for (int i = 0; i < 4; i++) {
    set_angular_velocities[i] = set_velocities[i] / (wheel_radius_);
  }

  SetDesiredAngularVelocities(set_angular_velocities);
}

void FourWheelOmni::EnableControl(void) {
  for (int i = 0; i < 4; i++) {
    wheel_controller_[i]->pid_->reset();
    wheel_controller_[i]->pid_->setInterval((double)control_cycle_us / 1000000);
    wheel_controller_[i]->ChangeControl(WheelController::ControlState::ENABLE);
  }
}

void FourWheelOmni::DisableControl(void) {
  counter_ = 0;
  for (int i = 0; i < 4; i++) {
    // wheel_controller_[i]->pid_[i].reset();
    wheel_controller_[i]->pid_[i].setInterval((double)control_cycle_us /
                                              1000000);
    wheel_controller_[i]->ChangeControl(WheelController::ControlState::DISABLE);
  }
}

double FourWheelOmni::GetCurrentSpeed(void) {
  return hypot(current_velocity_[0], current_velocity_[1]);
}

void FourWheelOmni::AddConfigurationCorrector(
    std::vector<DigitalIn *> conditions, int *fixed_point, int value) {
  ConfigurationCorrector cc(conditions, fixed_point, value);
  configuration_correctors_.push_back(cc);
}
