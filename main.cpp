#include "mbed.h"
#include <array>
#include <cmath>

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

Ticker ticker;
const int tread_width = 440;
const int half_of_thread_width = tread_width / 2;
const double inverse_of_tread_width = 1.0f / tread_width;
std::array<float, 3> current_configuration = {0, 0,
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
void MoveFourWheelOmni(float x, float y, float theta, float kx, float ky, float ktheta) {
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

float RestrictMinusPitoPi(float x) {
    float res = fmod((x + M_PI), 2 * M_PI) - M_PI;
    return res;
}

int main(void) {
  ticker.attach_us(&UpdateEncoders, interrupt_period_us);

  // x[mm], y[mm], theta[rad]
  std::array<float, 3> desired_configuration = {400, 1200, 1.57};
  // Supply Voltage: 7.89 V params
  float kp_rho = 0.00038;
  float kp_alpha = 0.04;
  float kp_beta = -0.013;
  float rho = numeric_limits<float>::max();
  float alpha, beta;
  float x_error, y_error;
  float v, omega;
  while (rho > 30) {
    GetCurrentConfiguration();
    x_error = desired_configuration[0] - current_configuration[0];
    y_error = desired_configuration[1] - current_configuration[1];
    rho = hypot(x_error, y_error);
    alpha = atan2(y_error, x_error) - current_configuration[2];
    beta = desired_configuration[2] - (current_configuration[2] + alpha);
    alpha = RestrictMinusPitoPi(alpha);
    beta = RestrictMinusPitoPi(beta);
    v = kp_rho * rho;
    omega = kp_alpha * alpha + kp_beta * beta;
    if ((alpha < - M_PI / 2) || (M_PI / 2 < alpha)) {
        v = -v;
    }


    MoveFourWheelOmni(v * cos(current_configuration[2]), v * sin(current_configuration[2]), -omega, 1, 1, 1);

    printf("%f\t%f\t%f\t", v * cos(current_configuration[2]), v * sin(current_configuration[2]), (omega/2/M_PI)*360);
    for (int i = 0; i < 3; i++) {
      printf("%f\t", current_configuration[i]);
    }
    printf("\n");
  }
}