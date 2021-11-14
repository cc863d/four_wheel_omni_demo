#include "Encoder.h"
#include "mbed.h"

int interrupt_period_us = 200;
Encoder encoders[] = {
    Encoder(PC_12, PC_10, 256, 102, interrupt_period_us),
    Encoder(PA_14, PA_15, 256, 102, interrupt_period_us),
    Encoder(PB_7, PC_13, 256, 102, interrupt_period_us),
    Encoder(PC_2, PC_3, 256, 102, interrupt_period_us),
};
Ticker ticker;

void UpdateEncoders(void) {
  for (int i = 0; i < 4; i++) {
    encoders[i].Counter();
  }
}

int main(void) {
  ticker.attach_us(&UpdateEncoders, interrupt_period_us);
  while (true) {
    for (int i = 0; i < 4; i++) {
      printf("%f\t", encoders[i].Distance());
    }
    printf("\n");
  }
}