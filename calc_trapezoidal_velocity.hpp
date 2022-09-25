#ifndef CALC_TRAPEZOIDAL_VELOCITY_HPP
#define CALC_TRAPEZOIDAL_VELOCITY_HPP

#include <array>
#include <math.h>


/// @param 移動距離[mm]，最大速度[mm/s]，最大加速度[mm/s/s]，初速度，終速度，周期
/// @return 加速期間長 巡航期間長, 減速期間長, （調整した）巡航速度, 加速時の加速度, 減速時の加速度
/// @detail https://tajimarobotics.com/quantization-acceleration-limited-feed-profile-2/
std::array<double, 6> CalcTrapezoidalVelocity(double distance, double max_velocity, double max_accel,
                                   double initial_velocity, double terminal_velocity, double dt)
{
    std::array<double, 6> res;

    // 加速期間長
    res.at(0) = (max_velocity - initial_velocity) / max_accel;
    // 減速期間長
    res.at(2) = (max_velocity - terminal_velocity) / max_accel;

    // 巡航時に進む距離=移動距離-加速時と減速時に進む距離
    double tmp_distance = distance - (max_velocity + initial_velocity) * res.at(0) * 0.5 - (max_velocity + terminal_velocity) * res.at(2) * 0.5;
    // 巡航期間長
    res.at(1) = tmp_distance / max_velocity;

    // 距離が短すぎたり，速度が大きすぎたりで
    // 三角形になってしまう場合
    if (res.at(1) <= 0)
    {
        // https://tajimarobotics.com/acceleration-limited-feed-profile-no-cruise/
        double peak_velocity;
        peak_velocity = sqrt(
            (2 * max_accel * distance + pow(initial_velocity, 2) + pow(terminal_velocity, 2)) * 0.5);
        res.at(0) = (peak_velocity - initial_velocity) / max_accel;
        res.at(2) = (peak_velocity - terminal_velocity) / max_accel;
        res.at(1) = 0;
    }

    // リンク先における量子化
    // 換言すれば制御周期の倍数になるように各期間長を調整
    for (int i = 0; i < 3; i++)
    {
        res.at(i) = ceil(res.at(i) / dt) * dt;
    }

    // 大きくなった各期間長にあわせて速度をや加速度を少し落とす
    res.at(3) = (2 * distance - initial_velocity * res.at(0) - terminal_velocity * res.at(2))
                / (res.at(0) + 2 * res.at(1) + res.at(2));
    // リンク先誤植注意
    res.at(4) = (res.at(3) - initial_velocity) / res.at(0);
    // 減速なので
    res.at(5) = (terminal_velocity - res.at(3) ) / res.at(2);


    return res;
}

#endif