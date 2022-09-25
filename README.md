

# 角速度制御
```cpp
#include "mbed.h"
#include "wheel/wheel_controller.hpp"

Encoder encoder(PC_12, PC_10, 256, 102, 200);
MDD10A md(PA_7, PB_6);
PID pid(0.3, 0.07, 0.01, 10);
WheelController wctrllr(&encoder, &md, &pid, 10);

Ticker ticker;
Ticker ticker2;
int main() {
  ticker.attach(callback(&wctrllr, &WheelController::ExecuteOneCycle), 10ms);
  ticker2.attach(callback(&encoder, &Encoder::Counter), 200us);
  while (true) {
    wctrllr.ApplyVoltageRate(-0.2); // 手動を想定
    thread_sleep_for(1000);
    wctrllr.ChangeControl(WheelController::ControlState::ENABLE);
    wctrllr.SetDesiredValue(3.14159); // 自動を想定
    thread_sleep_for(2000);
    wctrllr.SetDesiredValue(-3.14159);
    thread_sleep_for(5000);
    wctrllr.SetDesiredValue(0);
    thread_sleep_for(1000);
    wctrllr.ChangeControl(WheelController::ControlState::DISABLE);
    thread_sleep_for(1000);
    wctrllr.ApplyVoltageRate(0.2);
  }
}
```


# 手動制御
```cpp
#include "mbed.h"

#include "four_wheel_omni.hpp"
#include "manual_controller.hpp"
#include "ps3_pin_interrupt.hpp"
#include "wheel/wheel_controller.hpp"
#include <array>


PS3 ps3(PA_0, PA_1);
MotorDriver **mtrdrvs;
void CreateMotorDrivers() {
  mtrdrvs = new MotorDriver *[4];
  mtrdrvs[0] = new MDD10A(PA_7, PB_6);
  mtrdrvs[1] = new MDD10A(PB_10, PA_8);
  mtrdrvs[2] = new MDD10A(PA_9, PC_7);
  mtrdrvs[3] = new MDD10A(PA_6, PA_5);
}
int main() {
  CreateMotorDrivers();
  FourWheelOmni veh(mtrdrvs);
  std::array<double, 4> set_pwm;
  std::pair<double, double> right_joystick;
  std::pair<double, double> left_joystick;

  ManualController manual_controller;
  while (true) {

    right_joystick.first = ps3.getAnalogHat(Lx);
    right_joystick.second = 128 - ps3.getAnalogHat(Ly);
    left_joystick.first = 128 - ps3.getAnalogHat(Rx);
    left_joystick.second = ps3.getAnalogHat(Ry);
    set_pwm =
        manual_controller.CalculatePwm(right_joystick, left_joystick);
    veh.SetRateVoltage(set_pwm);
    printf("%f\t%f\t", right_joystick.first, right_joystick.second);
	printf("\n");
  }
}

```




# 手動制御（9 軸センサによる角度補正あり）
```cpp
#include "mbed.h"

#include "four_wheel_omni.hpp"
#include "manual_controller.hpp"
#include "ps3_pin_interrupt.hpp"
#include "wheel/wheel_controller.hpp"
#include <array>


PS3 ps3(PA_0, PA_1);
BNO055 ahrs(PB_9, PB_8);

MotorDriver **mtrdrvs;
void CreateMotorDrivers() {
  mtrdrvs = new MotorDriver *[4];
  mtrdrvs[0] = new MDD10A(PA_7, PB_6);
  mtrdrvs[1] = new MDD10A(PB_10, PA_8);
  mtrdrvs[2] = new MDD10A(PA_9, PC_7);
  mtrdrvs[3] = new MDD10A(PA_6, PA_5);
}
int main() {
  CreateMotorDrivers();
  FourWheelOmni veh(mtrdrvs);
  veh.AddAhrs(&ahrs);
  std::array<double, 4> set_pwm;
  std::pair<double, double> right_joystick;
  std::pair<double, double> left_joystick;

  ManualController manual_controller;
  int count = 0;
  while (true) {
    printf("%f\t%f\t", right_joystick.first, right_joystick.second);
    veh.ahrs_->get_angles();
    veh.current_configuration_[2] =
        -((veh.ahrs_->euler.yaw > 180) ? -(360 - veh.ahrs_->euler.yaw)
                                       : veh.ahrs_->euler.yaw);
    right_joystick.first = ps3.getAnalogHat(Lx);
    right_joystick.second = 128 - ps3.getAnalogHat(Ly);
    left_joystick.first = 128 - ps3.getAnalogHat(Rx);
    left_joystick.second = ps3.getAnalogHat(Ry);

    // 回っている間は目標角度を変え続ける
    if (left_joystick.first > 64 || left_joystick.first < 64) {
      veh.desired_yaw = veh.current_configuration_[2];
    }
    // 止まったら目標角度を動かさない
    if (right_joystick.first == 64 && right_joystick.second == 64) {
      count++;
      if (count > 20) {
        veh.desired_yaw = veh.current_configuration_[2];
        count = 0;
      }
    } else {
      count = 0;
    }
    // 以上目標角度の計算

    set_pwm =
        manual_controller.CalculatePwm(right_joystick, left_joystick, veh);
    veh.SetRateVoltage(set_pwm);
    printf("%f %f\n", veh.current_configuration_[2], veh.desired_yaw);
  }
}

```

# 自己位置推定
```cpp
#include "mbed.h"

#include "four_wheel_omni.hpp"
#include "manual_controller.hpp"
#include "ps3_pin_interrupt.hpp"
#include "wheel/wheel_controller.hpp"
#include <array>


PS3 ps3(PA_0, PA_1);
BNO055 ahrs(PB_9, PB_8);

MotorDriver **mtrdrvs;
void CreateMotorDrivers() {
  mtrdrvs = new MotorDriver *[4];
  mtrdrvs[0] = new MDD10A(PA_7, PB_6);
  mtrdrvs[1] = new MDD10A(PB_10, PA_8);
  mtrdrvs[2] = new MDD10A(PA_9, PC_7);
  mtrdrvs[3] = new MDD10A(PA_6, PA_5);
}


/**** WHEEL CONTROLLER ****/
WheelController **wctrlrs;
Encoder **rencs;
PID **pids;
int enc_interrupt_period = 200;
int diameter_drive_wheel = 102;
int enc_ppr_drive_wheel = 256;
int control_cycle_ms = 1;

void CreateWheelControllers() {
  CreateMotorDrivers();
  rencs = new Encoder *[4];
  rencs[3] = new Encoder(PC_2, PC_3, enc_ppr_drive_wheel, diameter_drive_wheel,
                         enc_interrupt_period);
  rencs[2] = new Encoder(PB_7, PC_13, enc_ppr_drive_wheel, diameter_drive_wheel,
                         enc_interrupt_period);
  rencs[1] = new Encoder(PA_14, PA_15, enc_ppr_drive_wheel,
                         diameter_drive_wheel, enc_interrupt_period);
  rencs[0] = new Encoder(PC_10, PC_12, enc_ppr_drive_wheel,
                         diameter_drive_wheel, enc_interrupt_period);

  pids = new PID *[4];
  pids[0] = new PID(0.3, 0.07, 0.0, control_cycle_ms * 1000);
  pids[1] = new PID(0.3, 0.07, 0.0, control_cycle_ms * 1000);
  pids[2] = new PID(0.3, 0.07, 0.0, control_cycle_ms * 1000);
  pids[3] = new PID(0.3, 0.07, 0.0, control_cycle_ms * 1000);

  wctrlrs = new WheelController *[4];
  for (int i = 0; i < 4; i++) {
    wctrlrs[i] =
        new WheelController(rencs[i], mtrdrvs[i], pids[i], control_cycle_ms);
  }
}
/**** WHEEL CONTROLLER ****/



int main() {
  CreateMotorDrivers();
  FourWheelOmni veh(mtrdrvs);

  // WheelController を追加すると自己位置を計測する  
  CreateWheelControllers();
  veh.AddWheelController(wctrlrs);

  std::array<double, 4> set_pwm;
  std::pair<double, double> right_joystick;
  std::pair<double, double> left_joystick;

  ManualController manual_controller;
  int count = 0;
  while (true) {

    right_joystick.first = ps3.getAnalogHat(Lx);
    right_joystick.second = 128 - ps3.getAnalogHat(Ly);
    left_joystick.first = 128 - ps3.getAnalogHat(Rx);
    left_joystick.second = ps3.getAnalogHat(Ry);

    set_pwm =
        manual_controller.CalculatePwm(right_joystick, left_joystick);
    veh.SetRateVoltage(set_pwm);

    printf("%f\t", veh.current_configuration_[0]);
    printf("%f\t", veh.current_configuration_[1]);
    printf("%f\t", hypot(veh.current_velocity_[0], veh.current_velocity_[1]));
    printf("%f\t", veh.current_configuration_[2]);
    printf("\n");
  }
}

```




# 自動走行
2022年9月機体で特に未テスト！
```cpp
#include "mbed.h"

#include "calc_desired_velocity.hpp"
#include "calc_trapezoidal_velocity.hpp"
#include "cubic_spline_planner.hpp"
#include "four_wheel_omni.hpp"
#include "target_cource.hpp"
#include "wheel/wheel_controller.hpp"
#include <cstdio>

MotorDriver **mtrdrvs;
WheelController **wctrlrs;
Encoder **rencs;
PID **pids;
int enc_interrupt_period = 200;
int diameter_drive_wheel = 102;
int enc_ppr_drive_wheel = 256;
int control_cycle_ms = 1;

void CreateMotorDrivers() {
  mtrdrvs = new MotorDriver *[4];
  mtrdrvs[0] = new MDD10A(PA_7, PB_6);
  mtrdrvs[1] = new MDD10A(PA_6, PA_5);
  mtrdrvs[2] = new MDD10A(PA_9, PC_7);
  mtrdrvs[3] = new MDD10A(PB_10, PA_8);
}

void CreateWheelControllers() {
  CreateMotorDrivers();
  rencs = new Encoder *[4];
  rencs[0] = new Encoder(PC_10, PC_12, enc_ppr_drive_wheel,
                         diameter_drive_wheel, enc_interrupt_period);
  rencs[1] = new Encoder(PC_2, PC_3, enc_ppr_drive_wheel, diameter_drive_wheel,
                         enc_interrupt_period);
  rencs[2] = new Encoder(PB_7, PC_13, enc_ppr_drive_wheel, diameter_drive_wheel,
                         enc_interrupt_period);
  rencs[3] = new Encoder(PA_14, PA_15, enc_ppr_drive_wheel,
                         diameter_drive_wheel, enc_interrupt_period);

  pids = new PID *[4];
  pids[0] = new PID(0.3, 0.07, 0.0, control_cycle_ms * 1000);
  pids[1] = new PID(0.3, 0.07, 0.0, control_cycle_ms * 1000);
  pids[2] = new PID(0.3, 0.07, 0.0, control_cycle_ms * 1000);
  pids[3] = new PID(0.3, 0.07, 0.0, control_cycle_ms * 1000);

  wctrlrs = new WheelController *[4];
  for (int i = 0; i < 4; i++) {
    wctrlrs[i] =
        new WheelController(rencs[i], mtrdrvs[i], pids[i], control_cycle_ms);
  }
}

BNO055 ahrs(PB_9, PB_8);
void AhrsSetup(void) {
  ahrs.reset();
  ahrs.setmode(OPERATION_MODE_NDOF);
  ahrs.get_calib();
}


int main() {

  std::vector<std::array<float, 2>> points =
      CublicSplinePlan2D({{0, 0}, {650, 500}, {0, 1230}, {650, 1380}});
  TargetCourse trajectory(points);
//   for (auto p : points) {
//     printf("%f,%f\n", p[0], p[1]);
//   }

  CreateWheelControllers();
  AhrsSetup();
  FourWheelOmni veh(mtrdrvs);
  veh.AddWheelController(wctrlrs);

  double syuki = 0.001; // 目標速度を変更する周期
  std::array<int, 2> distances = trajectory.CalcTotalDistance();
  std::array<double, 6> velprofile_x =
      CalcTrapezoidalVelocity(distances.at(0), 800, 100, 0, 0, syuki);
  std::array<double, 6> velprofile_y =
      CalcTrapezoidalVelocity(distances.at(1), 800, 100, 0, 0, syuki);
  std::array<std::array<double, 6>, 2> velprofile_xy = {velprofile_x,
                                                        velprofile_y};

  double vols[4] = {0, 0, 0, 0};
  veh.SetDesiredAngularVelocities(vols);
  veh.EnableControl();


  while (true) {
    int ind = trajectory.SearchTargetIndex(veh.current_configuration_[0],
                                           veh.current_configuration_[1],
                                           veh.GetCurrentSpeed());
    std::array<double, 3> desired_velocity = CalcDesiredVelocity(
        veh, trajectory.points_.at(ind), 0, velprofile_xy, syuki);
    veh.SetDesiredVelocities(desired_velocity);

    veh.ahrs_->get_angles();
    veh.current_configuration_[2] =
        (ahrs.euler.yaw > 180) ? -(360 - ahrs.euler.yaw) : ahrs.euler.yaw;

    printf("%f\t", veh.current_configuration_[0]);
    printf("%f\t", veh.current_configuration_[1]);
    printf("%f\t", hypot(veh.current_velocity_[0], veh.current_velocity_[1]));
    printf("%f\t", veh.current_configuration_[2]);
    printf("\n");
    ThisThread::sleep_for(30ms);
  }
}

```