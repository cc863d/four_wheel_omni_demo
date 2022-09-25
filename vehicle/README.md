
# FourWheelOmni 動作確認
```cpp
#include "four_wheel_omni.hpp"
#include "mbed.h"
#include "wheel/wheel_controller.hpp"
#include <array>

MotorDriver *mtrdrvs[4];

void setup() {
  mtrdrvs[0] = new MDD10A(PA_9, PC_7);
  mtrdrvs[1] = new MDD10A(PA_6, PA_5);
  mtrdrvs[2] = new MDD10A(PB_10, PA_8);
  mtrdrvs[3] = new MDD10A(PA_7, PB_6);

}

int main() {
  setup();

  FourWheelOmni veh(mtrdrvs);

  std::array<double, 4> vols = {0.5, 0.5, 0.5, 0.5};
  while (true) {
    veh.SetRateVoltage(vols);
  }
}
```

# FourWheelOmni 各車輪速度制御追加
```cpp

#include "four_wheel_omni.hpp"
#include "mbed.h"
#include "wheel/wheel_controller.hpp"
#include <array>

MotorDriver **mtrdrvs;
WheelController **wctrlrs;
Encoder **rencs;
PID **pids;
int enc_interrupt_period = 200;
int diameter_drive_wheel = 102;
int enc_ppr_drive_wheel = 256;
int control_cycle_ms = 1;

void setup() {
  mtrdrvs = new MotorDriver *[4];
  mtrdrvs[0] = new MDD10A(PA_9, PC_7);
  mtrdrvs[1] = new MDD10A(PA_6, PA_5);
  mtrdrvs[2] = new MDD10A(PB_10, PA_8);
  mtrdrvs[3] = new MDD10A(PA_7, PB_6);

  rencs = new Encoder *[4];
  rencs[0] = new Encoder(PC_10, PC_12, enc_ppr_drive_wheel,
                         diameter_drive_wheel, enc_interrupt_period);
  rencs[1] = new Encoder(PA_15, PA_14, enc_ppr_drive_wheel,
                         diameter_drive_wheel, enc_interrupt_period);
  rencs[2] = new Encoder(PC_13, PB_7, enc_ppr_drive_wheel, diameter_drive_wheel,
                         enc_interrupt_period);
  rencs[3] = new Encoder(PC_2, PC_3, enc_ppr_drive_wheel, diameter_drive_wheel,
                         enc_interrupt_period);
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

int main() {
  setup();
  FourWheelOmni veh(mtrdrvs);
  veh.AddWheelController(wctrlrs);
  double vols[4] = {7, 7, 7, 7};
  veh.SetDesiredAngularVelocities(vols);
  veh.EnableControl();
    
  while (true) {
    printf("%f\n", veh.wheel_controller_[0]->enc_->GetAngularVelocity());
  }
}


```