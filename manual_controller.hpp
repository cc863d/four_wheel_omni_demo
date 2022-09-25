#ifndef MANUAL_CONTROLLER
#define MANUAL_CONTROLLER

#include "cmath"
#include "four_wheel_omni.hpp"
#include "tuple"

class ManualController {

public:
  std::tuple<double, double, double>
  CalculateConfiguration(std::pair<double, double> input_right,
                         std::pair<double, double> input_left);
  std::tuple<double, double, double>
  CalculateConfiguration(std::pair<double, double> input_right,
                         std::pair<double, double> input_left,
                         FourWheelOmni &vehicle);
  std::array<double, 4> CalculatePwm(std::pair<double, double> input_right,
                                     std::pair<double, double> input_left);
  std::array<double, 4> CalculatePwm(std::pair<double, double> input_right,
                                     std::pair<double, double> input_left,
                                     FourWheelOmni &vehicle);
  double power_rate_ = 0.5; // �������̃p���[�H�ɓ����邽�߂̕ϐ�[0,1]

private:
  // �Ƃ肠���� pair
  // ���g�������C�����ǂ݂₷���Ȃ���x�Ŏ����͏��Ȃ��C������
  std::pair<double, double> input_a_joystick_;  // first: x,  second: y
  std::pair<double, double> input_b_joystick_;  // "
  std::pair<double, double> scaled_a_joystick_; // "
  std::pair<double, double> scaled_b_joystick_; // "

  const double input_upper_limit_ = 127 + 1; // �Ƃ肠���� + 1
  const double input_lower_limit_ = 0;
  const double scaled_upper_limit_ = 1;
  const double scaled_lower_limit_ = -1;

  std::pair<double, double> NormalizeVec(std::pair<double, double> input,
                                         double input_upper_limit,
                                         double input_lower_limit,
                                         double output_upper_limit,
                                         double output_lower_limit);
  std::pair<double, double> LimitIntoDiamond(std::pair<double, double> input);
};

std::pair<double, double> ManualController::NormalizeVec(
    std::pair<double, double> input, double input_upper_limit,
    double input_lower_limit, double output_upper_limit,
    double output_lower_limit) {
  std::pair<double, double> output;
  output.first = (input.first - input_lower_limit) *
                     (output_upper_limit - output_lower_limit) /
                     (input_upper_limit - input_lower_limit) +
                 output_lower_limit;
  output.second = (input.second - input_lower_limit) *
                      (output_upper_limit - output_lower_limit) /
                      (input_upper_limit - input_lower_limit) +
                  output_lower_limit;

  if (output.first > output_upper_limit)
    output.first = output_upper_limit;
  if (output.second > output_upper_limit)
    output.second = output_upper_limit;
  if (output.first < output_lower_limit)
    output.first = output_lower_limit;
  if (output.second < output_lower_limit)
    output.second = output_lower_limit;

  return output;
}

std::pair<double, double>
ManualController::LimitIntoDiamond(std::pair<double, double> input) {
  // https://tomirobo.esa.io/posts/227
  // 1 >= |x|+|y| �̗̈�Ɏ��߂��l��Ԃ�
  // 1 = |x| + |y| ���_�C�A�����h�^�Ȃ̂� Limit into Diamond
  std::pair<double, double> output;
  if (1 >= fabs(input.first) + fabs(input.second)) {
    output = input;
  } else {
    // �����������ꂢ�ɂȂ�܂���
    double bx = 0, by = 0;
    double cx = 0, cy = 0;
    double r;
    if (0 <= input.first && 0 <= input.second) {
      // ���ی�
      bx = 0;
      by = 1;
      cx = 1;
      cy = 0;
    } else if (input.first < 0 && 0 <= input.second) {
      // ���ی�
      bx = 0;
      by = 1;
      cx = -1;
      cy = 0;
    } else if (input.first < 0 && input.first < 0) {
      // ��O�ی�
      bx = -1;
      by = 0;
      cx = 0;
      cy = -1;
    } else if (0 <= input.first && input.second < 0) {
      // ��l�ی�
      bx = 1;
      by = 0;
      cx = 0;
      cy = -1;
    }
    r = (bx * (cy - by) - by * (cx - bx)) /
        (input.first * (cy - by) - input.second * (cx - bx));
    output.first = r * input.first;
    output.second = r * input.second;
  }

  return output;
}

std::tuple<double, double, double>
ManualController::CalculateConfiguration(std::pair<double, double> input_right,
                                         std::pair<double, double> input_left) {
  std::tuple<double, double, double> output;
  // input_a: xy �ړ��̎w�����e
  // input_b: ����̎w�����e
  input_a_joystick_ = input_right;
  input_b_joystick_ = input_left;

  // y ���͍��W�̌������t�Ȃ̂�
  input_a_joystick_.second = input_upper_limit_ - input_a_joystick_.second;
  input_b_joystick_.second = input_upper_limit_ - input_b_joystick_.second;

  // xy �ړ��̎w���̐��`�ϊ�
  scaled_a_joystick_ =
      NormalizeVec(input_a_joystick_, input_upper_limit_, input_lower_limit_,
                   scaled_upper_limit_, scaled_lower_limit_);
  scaled_a_joystick_ = LimitIntoDiamond(scaled_a_joystick_);
  // ����w���̐��`�ϊ�
  scaled_b_joystick_ =
      NormalizeVec(input_b_joystick_, input_upper_limit_, input_lower_limit_,
                   scaled_upper_limit_, scaled_lower_limit_);
  // esa �̂܂Ƃ߂� 5
  scaled_a_joystick_ =
      NormalizeVec(scaled_a_joystick_, scaled_upper_limit_, scaled_lower_limit_,
                   (1 - fabs(scaled_b_joystick_.first)),
                   -(1 - fabs(scaled_b_joystick_.first)));

  // + b_second �̍������񒆐S�Ɋ֗^����H�H
  std::get<0>(output) =
      (scaled_a_joystick_.first + scaled_b_joystick_.second) * power_rate_;
  std::get<1>(output) =
      (scaled_a_joystick_.second + scaled_b_joystick_.second) * power_rate_;
  std::get<2>(output) = scaled_b_joystick_.first * power_rate_;

  return output;
}

std::tuple<double, double, double>
ManualController::CalculateConfiguration(std::pair<double, double> input_right,
                                         std::pair<double, double> input_left,
                                         FourWheelOmni &vehicle) {
  std::tuple<double, double, double> output;
  // input_a: xy �ړ��̎w�����e
  // input_b: ����̎w�����e
  input_a_joystick_ = input_right;
  input_b_joystick_ = input_left;

  // y ���͍��W�̌������t�Ȃ̂�
  input_a_joystick_.second = input_upper_limit_ - input_a_joystick_.second;
  input_b_joystick_.second = input_upper_limit_ - input_b_joystick_.second;

  // xy �ړ��̎w���̐��`�ϊ�
  scaled_a_joystick_ =
      NormalizeVec(input_a_joystick_, input_upper_limit_, input_lower_limit_,
                   scaled_upper_limit_, scaled_lower_limit_);
  scaled_a_joystick_ = LimitIntoDiamond(scaled_a_joystick_);
  // ����w���̐��`�ϊ�
  scaled_b_joystick_ =
      NormalizeVec(input_b_joystick_, input_upper_limit_, input_lower_limit_,
                   scaled_upper_limit_, scaled_lower_limit_);
  // esa �̂܂Ƃ߂� 5
  scaled_a_joystick_ =
      NormalizeVec(scaled_a_joystick_, scaled_upper_limit_, scaled_lower_limit_,
                   (1 - fabs(scaled_b_joystick_.first)),
                   -(1 - fabs(scaled_b_joystick_.first)));

  double yaw_error = vehicle.desired_yaw - vehicle.current_configuration_[2];
  // + b_second �̍������񒆐S�Ɋ֗^����H�H
  std::get<0>(output) = (scaled_a_joystick_.first) * power_rate_;
  std::get<1>(output) = (scaled_a_joystick_.second) * power_rate_;
  if (fabs(scaled_b_joystick_.first) > __FLT_EPSILON__ ||
      (fabs(scaled_a_joystick_.first) <= __FLT_EPSILON__ &&
       fabs(scaled_a_joystick_.second) <= __FLT_EPSILON__)) {
    // 回転したい時および操作していないときは角度の誤差修正しない
    std::get<2>(output) = (scaled_b_joystick_.first) * power_rate_;
  } else {
    // そうでないときは角度を修正
    std::get<2>(output) =
        (scaled_b_joystick_.first - yaw_error * -0.01) * power_rate_;
  }

  return output;
}

std::array<double, 4>
ManualController::CalculatePwm(std::pair<double, double> input_right,
                               std::pair<double, double> input_left) {
  std::array<double, 4> output;

  std::tuple<double, double, double> tmp_configuration;
  tmp_configuration = CalculateConfiguration(input_right, input_left);

  output[0] = -std::get<0>(tmp_configuration) + std::get<1>(tmp_configuration) -
              std::get<2>(tmp_configuration);
  output[1] = -std::get<0>(tmp_configuration) - std::get<1>(tmp_configuration) -
              std::get<2>(tmp_configuration);
  output[2] = +std::get<0>(tmp_configuration) - std::get<1>(tmp_configuration) -
              std::get<2>(tmp_configuration);
  output[3] = +std::get<0>(tmp_configuration) + std::get<1>(tmp_configuration) -
              std::get<2>(tmp_configuration);

  return output;
}

std::array<double, 4>
ManualController::CalculatePwm(std::pair<double, double> input_right,
                               std::pair<double, double> input_left,
                               FourWheelOmni &vehicle) {
  std::array<double, 4> output;

  std::tuple<double, double, double> tmp_configuration;
  tmp_configuration = CalculateConfiguration(input_right, input_left, vehicle);

  output[0] = -std::get<0>(tmp_configuration) + std::get<1>(tmp_configuration) -
              std::get<2>(tmp_configuration);
  output[1] = -std::get<0>(tmp_configuration) - std::get<1>(tmp_configuration) -
              std::get<2>(tmp_configuration);
  output[2] = +std::get<0>(tmp_configuration) - std::get<1>(tmp_configuration) -
              std::get<2>(tmp_configuration);
  output[3] = +std::get<0>(tmp_configuration) + std::get<1>(tmp_configuration) -
              std::get<2>(tmp_configuration);

  return output;
}
#endif