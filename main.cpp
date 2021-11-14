#include "mbed.h"
#include <array>

#include "Encoder.h"
#include "ps3_pin_interrupt.hpp"

int interrupt_period_us = 200;
Encoder encoders[] = {
    Encoder(PC_12, PC_10, 256, 102, interrupt_period_us),
    Encoder(PA_14, PA_15, 256, 102, interrupt_period_us),
    Encoder(PB_7, PC_13, 256, 102, interrupt_period_us),
    Encoder(PC_2, PC_3, 256, 102, interrupt_period_us),
};
PwmOut pwm[] = {PwmOut(PC_7), PwmOut(PA_5), PwmOut(PA_8), PwmOut(PB_6)};
DigitalOut dir[] = {
    DigitalOut(PA_9), DigitalOut(PA_6), DigitalOut(PB_10),
    DigitalOut(PA_7) // direction is reversed to others
};

PS3 ps3(PA_0, PA_1);
//  constants using manual controll
float kx = 1.0f / 100.0f;
float ky = 1.0f / 100.0f;
float ktheta = 1.0f / 100.0f;

Ticker ticker;
const int tread_width = 440;
const int half_of_thread_width = tread_width / 2;
const double inverse_of_tread_width = 1.0f / tread_width;
std::array<double, 3> current_configuration = {0, 0,
                                               0}; // x[mm], y[mm], theta[rad]

void UpdateEncoders(void) {
  for (int i = 0; i < 4; i++) {
    encoders[i].Counter();
  }
}

void GetCurrentConfiguration(void) {
  // See forward kinematics of 4 wheel omni.
  // 0.7071f = 1 / sqrt(2)
  static float previous_distances[4] = {
      0, 0, 0, 0}; // It is initialized only once the first time.
  float delta_wheel_theta[4];
  for (int i = 0; i < 4; i++) {
    delta_wheel_theta[i] = encoders[i].Distance() - previous_distances[i];
  }
  float dx =
      0.5f *
      ((-0.7071f * delta_wheel_theta[0]) + (-0.7071f * delta_wheel_theta[1]) +
       (+0.7071f * delta_wheel_theta[2]) + (+0.7071f * delta_wheel_theta[3]));
  float dy =
      0.5f *
      ((-0.7071f * delta_wheel_theta[0]) + (+0.7071f * delta_wheel_theta[1]) +
       (+0.7071f * delta_wheel_theta[2]) + (-0.7071f * delta_wheel_theta[3]));

  current_configuration[0] +=
      dx * cos(current_configuration[2]) - dy * sin(current_configuration[2]);
  current_configuration[1] +=
      dx * sin(current_configuration[2]) + dy * cos(current_configuration[2]);
  current_configuration[2] =
      0.5f * ((inverse_of_tread_width * -encoders[0].Distance()) +
              (inverse_of_tread_width * -encoders[1].Distance()) +
              (inverse_of_tread_width * -encoders[2].Distance()) +
              (inverse_of_tread_width * -encoders[3].Distance()));
  for (int i = 0; i < 4; i++) {
    previous_distances[i] = encoders[i].Distance();
  }
}
void MoveFourWheelOmniByManual(float x, float y, float theta) {
  float v[4] = {0, 0, 0, 0};
  // See Inverse Kinematics of 4 wheel omni.
  v[0] = -kx * x + -ky * y + ktheta * theta;
  v[1] = -kx * x + +ky * y + ktheta * theta;
  v[2] = +kx * x + +ky * y + ktheta * theta;
  v[3] = +kx * x + -ky * y + ktheta * theta;
  for (int i = 0; i < 4; i++) {
    pwm[i].write(abs(v[i]));
    if (v[i] >= 0) {
      dir[i].write(0);
    } else {
      dir[i].write(1);
    }
    dir[3] = !(dir[3]);
  }
}

int main(void) {
  ticker.attach_us(&UpdateEncoders, interrupt_period_us);
  while (true) {
    GetCurrentConfiguration();
    MoveFourWheelOmniByManual((float)(ps3.getAnalogHat(Rx) - 64),
                              (float)(64 - ps3.getAnalogHat(Ry)),
                              (float)(ps3.getAnalogHat(Lx) - 64));
    for (int i = 0; i < 3; i++) {
      printf("%f\t", current_configuration[i]);
    }
    printf("\n");
  }
}