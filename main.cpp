#include "mbed.h"
#include <array>
#include <cmath>
#include <limits>

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
void MoveFourWheelOmni(float x, float y, float theta, float kx, float ky,
                       float ktheta) {
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

std::vector<std::array<int, 2>>
LinearPlan(std::array<int, 2> const start_point,
           std::array<int, 2> const goal_point) {
  std::vector<std::array<int, 2>> res;
  const int point_num = 100; // 100という点数は適当
  std::array<double, 2> point = {double(start_point.at(0)),
                                 double(start_point.at(1))};
  double x_increment =
      (goal_point.at(0) - start_point.at(0)) / (double)point_num;
  double y_increment =
      (goal_point.at(1) - start_point.at(1)) / (double)point_num;
  for (int i = 0; i < point_num; i++) {
    res.push_back({(int)point.at(0), (int)point.at(1)});
    point.at(0) += x_increment;
    point.at(1) += y_increment;
  }
  return res;
}

int SearchTargetIndex(const std::vector<std::array<int, 2>> &points, int x,
                      int y, double v, int &old_nearest_point_index_) {
  // Why can the index decrease?
  int tmp_index = old_nearest_point_index_;
  double distance_this_index =
      hypot(x - points.at(tmp_index).at(0), y - points.at(tmp_index).at(1));
  for (int i = tmp_index + 1; i < (int)points.size(); i++) {
    int dx = x - points.at(i).at(0);
    int dy = y - points.at(i).at(1);
    double distance = hypot(dx, dy);
    if (distance_this_index < distance) {
      break;
    }
    tmp_index = ((i + 1) < (int)points.size()) ? (i + 1) : i;
    distance_this_index = distance;
  }
  old_nearest_point_index_ = tmp_index;
  double look_forward_distance = 80 + v * 0.001 * 300;
  while (look_forward_distance > hypot(x - points.at(tmp_index).at(0),
                                       y - points.at(tmp_index).at(1))) {
    if ((tmp_index + 1) >= (int)points.size()) {
      break;
    }
    tmp_index += 1;
  }
  return max(old_nearest_point_index_, tmp_index);
}

int main(void) {
  ticker.attach_us(&UpdateEncoders, interrupt_period_us);
  // x[mm], y[mm], theta[rad]
  float v, vx, vy, vtheta;
  float desired_direction;
  float Kv = 0.002;
  float error = numeric_limits<float>::max();
  std::array<float, 3> desired_configuration = {0, 0,
                                                0}; // x[mm], y[mm], theta[rad]
  std::vector<std::array<int, 2>> desired_trajectory =
      LinearPlan({{0, 0}}, {{300, 0}});
  auto b = LinearPlan({{300, 0}}, {{300, 300}});
  auto c = LinearPlan({{300, 300}}, {{0, 300}});
  auto d = LinearPlan({{0, 300}}, {{0, 600}});
  auto e = LinearPlan({{0, 600}}, {{300, 600}});
  desired_trajectory.insert(desired_trajectory.end(), b.begin(), b.end());
  desired_trajectory.insert(desired_trajectory.end(), c.begin(), c.end());
  desired_trajectory.insert(desired_trajectory.end(), d.begin(), d.end());
  desired_trajectory.insert(desired_trajectory.end(), e.begin(), e.end());
  int index_nearest_point = 0, index_target_point = 0;
  while (true) {
    GetCurrentConfiguration();
    index_target_point =
        SearchTargetIndex(desired_trajectory, current_configuration[0],
                          current_configuration[1], v, index_nearest_point);
    desired_configuration[0] = desired_trajectory[index_target_point][0];
    desired_configuration[1] = desired_trajectory[index_target_point][1];
    error = hypot(desired_configuration[0] - current_configuration[0],
                  desired_configuration[1] - current_configuration[1]);
    v = Kv * error;
    desired_direction =
        atan2(desired_configuration[1] - current_configuration[1],
              desired_configuration[0] - current_configuration[0]);
    vx = v * cos(desired_direction);
    vy = v * sin(desired_direction);
    vtheta = 0;
    MoveFourWheelOmni(vx, vy, vtheta, 1, 1, 1);

    printf("%d\t%4.0f\t%4.0f\t%f\t%f\t%f\t", index_target_point,
           desired_configuration[0], desired_configuration[1], vx, vy, vtheta);
    for (int i = 0; i < 3; i++) {
      printf("%f\t", current_configuration[i]);
    }
    printf("\n");
  }
}