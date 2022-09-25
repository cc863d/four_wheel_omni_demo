#ifndef MBED_PS3_PIN_INTERRUPT_HPP
#define MBED_PS3_PIN_INTERRUPT_HPP

#include "mbed.h"
#include <cstdint>

#define START_ARRAY_NUM 13
#define SELECT_ARRAY_NUM 14

enum ButtonEnum {
  UP = 0x0001,
  DOWN = 0x0002,
  RIGHT = 0x0004,
  LEFT = 0x0008,
  TRIANGEL = 0x0010,
  CROSS = 0x0020,
  CIRCLE = 0x0040,
  SQUARE = 0x0100,
  L1 = 0x0200,
  L2 = 0x0400,
  R1 = 0x0800,
  R2 = 0x1000,
  START = 0x0003,
  SELECT = 0x000c,
};

#define CHAR_START 0x80 // 電文の先頭文字

enum AnalogHatEnum {
  /** Left joystick x-axis */
  Lx = 0,
  /** Left joystick y-axis */
  Ly = 1,
  /** Right joystick x-axis */
  Rx = 2,
  /** Right joystick y-axis */
  Ry = 3,
};
class PS3 {
public:
  UnbufferedSerial pic_device;
  uint8_t rx_busy_buffer[7]; // 受信中に一時的に受信データをためておくバッファ
  uint8_t rx_now; // 受信中の文字の位置
  int is_finished_receive = 0;

  uint8_t counters_button_continuous_press[16];
  int is_already_called_click[16];
  const int threshold_click_button = 3;

  void ps3_main() {
    char c;
    pic_device.read(&c, 1);
    if (c == CHAR_START) {
      rx_now = 0;
    } else if (rx_now == 6) {
      is_finished_receive = 1;
      memcpy(rx_buffer, rx_busy_buffer, sizeof(rx_buffer));
      // とりあえずチェックサムは無視

      for (int i = 0; i < 13; i++) {
        if ((((rx_buffer[0] << 8) + rx_buffer[1]) >> i) & 1) {
          counters_button_continuous_press[i] =
              min(counters_button_continuous_press[i] + 1, UINT8_MAX);
        } else {
          counters_button_continuous_press[i] = 0;
          is_already_called_click[i] = 0;
        }
      }

      // start
      if (counters_button_continuous_press[0] !=
              counters_button_continuous_press[1] ||
          counters_button_continuous_press[0] == 0) {
        counters_button_continuous_press[START_ARRAY_NUM] = 0;
        is_already_called_click[START_ARRAY_NUM] = 0;
      } else {
        counters_button_continuous_press[START_ARRAY_NUM] = min(
            counters_button_continuous_press[START_ARRAY_NUM] + 1, UINT8_MAX);
      }

      // select
      if (counters_button_continuous_press[2] !=
              counters_button_continuous_press[3] ||
          counters_button_continuous_press[2] == 0) {
        counters_button_continuous_press[SELECT_ARRAY_NUM] = 0;
        is_already_called_click[SELECT_ARRAY_NUM] = 0;
      } else {
        counters_button_continuous_press[SELECT_ARRAY_NUM] = min(
            counters_button_continuous_press[SELECT_ARRAY_NUM] + 1, UINT8_MAX);
      }

      rx_now = 0;
    } else {
      // CHAR_START を除いた最初のデータが buf[0] に入る
      rx_busy_buffer[rx_now] = c;
      rx_now = (rx_now + 1) % 7;
    }
  };

public:
  uint8_t
      rx_buffer[7]; //受信が完了し，値が確定している受信データをためるバッファ
  PS3(PinName tx, PinName rx);
  uint8_t getAnalogHat(AnalogHatEnum a) {
    // 4-7 バイト目にアナログの値は入ってる
    // が，1 バイト目はない（0x80固定なので省略）
    return (uint8_t)(rx_buffer[2 + a]);
  };
  /**
   * getPress(ButtonEnum b) はボタンが桜花されている間 true を返す
   * getClick(ButtonEnum b) はボタンが押下された後，一度だけ true を返す
   */
  bool getButtonPress(ButtonEnum b) {
    // 8: sizeof(rx_buffer[0])
    int16_t button_state = (rx_buffer[0] << 8) + rx_buffer[1];
    // start/select だけ2bit フラグがたっているので
    // SELECT ボタンがRIGHTやLEFT を優越する
    if ((button_state & (int16_t)SELECT) == (int16_t)SELECT &&
        (b == (int16_t)LEFT || b == (int16_t)RIGHT)) {
      // SELECT ボタンが押されていると考えられるときは
      // 右左ボタンは押されていないことにする
      return false;
    }
    if ((button_state & (int16_t)START) == (int16_t)START &&
        (b == (int16_t)UP || b == (int16_t)DOWN)) {
      // START ボタンが押されていると考えられるときは
      // 上下ボタンは押されていないことにする
      return false;
    }
    return ((button_state & b) == b);
  };

  bool getButtonClick(ButtonEnum b) {

    // START

    if (b == (int16_t)START && is_already_called_click[START_ARRAY_NUM] == 0) {
      if (counters_button_continuous_press[0] > threshold_click_button &&
          counters_button_continuous_press[0] ==
              counters_button_continuous_press[1]) {
        is_already_called_click[START_ARRAY_NUM] = 1;
        return true;
      } else {
        return false;
      }
    }
    // select

    if (b == (int16_t)SELECT &&
        (is_already_called_click[SELECT_ARRAY_NUM] == 0)) {
      if ((counters_button_continuous_press[2] > threshold_click_button) &&
          (counters_button_continuous_press[2] ==
           counters_button_continuous_press[3])) {
        is_already_called_click[SELECT_ARRAY_NUM] = 1;
        return true;
      } else {
        return false;
      }
    }

    // other
    int k;
    for (int i = 0; i < 16; i++) {
      if ((b >> i) & 1) {
        k = i;
        break;
      }
    }

    if ((is_already_called_click[k] == 0) &&
        (counters_button_continuous_press[k] > threshold_click_button)) {
      is_already_called_click[k] = 1;
      return true;
    }
    return false;
  };
};
PS3::PS3(PinName tx, PinName rx)
    : pic_device(tx, rx) // 初期化子リスト
{
  pic_device.baud(2400);
  pic_device.format(8, SerialBase::None, 1);
  pic_device.attach(callback(this, &PS3::ps3_main), UnbufferedSerial::RxIrq);
}

#endif
