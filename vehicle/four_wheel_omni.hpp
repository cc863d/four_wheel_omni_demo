#ifndef FOUR_OMNI_HPP
#define FOUR_OMNI_HPP

#include "mbed.h"

#include "BNO055.h"
#include "configuration_corrector.hpp"
#include "platform/mbed_thread.h"
#include "wheel_controller.hpp"
#include <array>

//
//      front
//  /2/         \3\
//
// left         right    
//
//  \1\         /0/
//       back
//


class FourWheelOmni {
public:
  enum class DriverType {
    SIMPLIFIED_SERIAL,
    DIR_PWM,
  };
  FourWheelOmni(MotorDriver *md[4]);
  void SetRateVoltage(std::array<double, 4> pwm);
  void SetDesiredAngularVelocities(double angular_velocities[4]);
  void SetDesiredVelocities(std::array<double, 3> desired_velocity_vehicle);
  void EnableControl(void); // 制御オンする
  void DisableControl(void);
  double GetCurrentSpeed(void);

  MotorDriver *md_[4];

  // For WheelController
  WheelController *wheel_controller_[4];

  int AddWheelController(WheelController * wheel_controller[4]);

  std::vector<ConfigurationCorrector> configuration_correctors_;
  BNO055 *ahrs_;
  //[0]:for odometry of x axis [1]: y axis
  Encoder *renc_odometry_xy_[2];
  void AddConfigurationCorrector(std::vector<DigitalIn *> conditions,
                                 int * fixed_point, int value);
  int AddAhrs(BNO055 *ahrs);
  int AddEncoderForOdometry(Encoder *renc_odometry_xy[2]);

  // For Thread
  Thread encoder_interrupter_thread_;
  Thread execute_one_cycle_interrupter_thread_;
  void EncoderInterrupterThreadDriver();
  void ExecuteControlForOneCycleThreadDriver();
  Ticker encoder_interrupter_;
  Ticker execute_one_cycle_interrupter_;
  void EncoderDrive();
  void ExecuteControlForOneCycle();

  // States
  double current_configuration_[3] = {}; // x[mm], y[mm], theta[deg]
  double desired_velocity_[3] = {};
  float desired_yaw = 0;
  double current_velocity_[2]; // x[mm/s] y[mm/s]
  double previous_wheel_distance[4] = {};
  uint32_t counter_ = 0; // 自動化に使う。enable を設定してから何cycle目か。

  

  // Constants
  int control_cycle_us = 1000;
  const uint32_t max_count_ = UINT32_MAX; // 1時間 ~ UINT32_MAX マイクロ秒
  int encoder_interrupt_time_us_ = 200;

  const int wheel_radius_ = 100;
  const int wheel_separation_width_ = 370 / 2;
  const int wheel_separation_length_ = 370 / 2;
};

#endif