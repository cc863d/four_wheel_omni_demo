# MDD10Aで角速度制御をするテスト
```cpp
#include "mbed.h"
#include "wheel/wheel_controller.hpp"

Encoder encoder(PC_12, PC_10, 256, 102, 200);
MDD10A md(PA_9, PC_7);
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
# MDDS30
```cpp
#include "mbed.h"
#include "wheel/wheel_controller.hpp"

UnbufferedSerial md_serial(PA_0, PA_1, 115200);
Encoder encoder(PC_12, PC_10, 256, 102, 200);
MDDS30 md(&md_serial, 1);
PID pid(0.3, 0.07, 0.01, 10);
WheelController wctrllr(&encoder, &md, &pid, 10);

Ticker ticker;
Ticker ticker2;
int main() {
  ticker.attach(callback(&wctrllr, &WheelController::ExecuteOneCycle), 10ms);
  ticker2.attach(callback(&encoder, &Encoder::Counter), 200us);
  while (true) {
    wctrllr.ApplyVoltageRate(-0.2);
    thread_sleep_for(1000);
    wctrllr.ChangeControl(WheelController::ControlState::ENABLE);
    wctrllr.SetDesiredValue(3.14159);
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

