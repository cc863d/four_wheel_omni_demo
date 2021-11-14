#include "Encoder.h"
#include "mbed.h"
#include <array>

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

Ticker ticker;
const double inverse_of_tread_width = 1.0f / 440;
std::array<double, 3> current_configuration = {0, 0, 0}; // x, y, theta

void UpdateEncoders(void) {
  for (int i = 0; i < 4; i++) {
    encoders[i].Counter();
  }
}

void GetCurrentConfiguration(void) {
  // See forward kinematics of 4 wheel omni.
  current_configuration[0] = 0.5f * ((-0.7071f * encoders[0].Distance()) +
                                     (-0.7071f * encoders[1].Distance()) +
                                     (+0.7071f * encoders[2].Distance()) +
                                     (+0.7071f * encoders[3].Distance()));
  current_configuration[1] = 0.5f * ((-0.7071f * encoders[0].Distance()) +
                                     (+0.7071f * encoders[1].Distance()) +
                                     (+0.7071f * encoders[2].Distance()) +
                                     (-0.7071f * encoders[3].Distance()));
  current_configuration[2] =
      0.5f * ((inverse_of_tread_width * encoders[0].Distance()) +
              (inverse_of_tread_width * encoders[1].Distance()) +
              (inverse_of_tread_width * encoders[2].Distance()) +
              (inverse_of_tread_width * encoders[3].Distance()));
}

int main(void) {
  ticker.attach_us(&UpdateEncoders, interrupt_period_us);

  for (int i = 0; i < 4; i++) {
    pwm[i] = 0.5f;
  }
  dir[0] = 1;
  //   dir[3] = 1; // why?

  while (true) {
    GetCurrentConfiguration();
    for (int i = 0; i < 3; i++) {
      printf("%f\t", current_configuration[i]);
    }
    printf("\n");
  }
}