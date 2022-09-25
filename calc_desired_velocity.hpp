    #ifndef CALC_DESIRED_VELOCITY_HPP
#define CALC_DESIRED_VELOCITY_HPP

#include <array>
#include "four_wheel_omni.hpp"

std::array<double, 3> CalcDesiredVelocity(FourWheelOmni const &vehicle,
                                               std::array<float, 2> const &target_point,
                                               double target_yaw,
                                               std::array<std::array<double, 6>, 2> const &velocity_profile_xy,
                                               double control_cycle)
{
    std::array<double, 3> res; // v_x, v_y, v_theta
    std::array<float, 2> vehicle_error = {
        float(target_point[0] - vehicle.current_configuration_[0]),
        float(target_point[1] - vehicle.current_configuration_[1])};
    // ��ｽx��ｽN��ｽg��ｽ��ｽ��ｽﾌ鯉ｿｽ��ｽ��ｽ
    double running_direction = atan2(target_point[1] - vehicle.current_configuration_[1],
                                     target_point[0] - vehicle.current_configuration_[0]);

    for (int i = 0; i < 2; i++)
    {
        // ��ｽx��ｽN��ｽg��ｽ��ｽ��ｽﾌ大き��ｽ��ｽ
        double desired_velocity;
        double elapsed_time = double(vehicle.counter_) / vehicle.control_cycle_us * 0.000001; // us -> s
        if (elapsed_time <= velocity_profile_xy.at(i).at(0))
        {
            // ��ｽ��ｽ��ｽ��ｽ��ｽ��ｽ��ｽ��ｽ
            desired_velocity = velocity_profile_xy.at(i).at(4) * elapsed_time;
        }
        else if (elapsed_time - velocity_profile_xy.at(i).at(0) <= velocity_profile_xy.at(i).at(1))
        {
            // ��ｽ��ｽ��ｽq��ｽ��ｽ��ｽ��ｽ
            desired_velocity = velocity_profile_xy.at(i).at(3);
        }
        else if (elapsed_time - velocity_profile_xy.at(i).at(0) - velocity_profile_xy.at(i).at(1) <= velocity_profile_xy.at(i).at(2))
        {
            // ��ｽ��ｽ��ｽ��ｽ��ｽ��ｽ��ｽ��ｽ
            desired_velocity = velocity_profile_xy.at(i).at(3) + velocity_profile_xy.at(i).at(5) * (elapsed_time - (velocity_profile_xy.at(i).at(0) + velocity_profile_xy.at(i).at(1)));
        }
        else
        {
            desired_velocity = 0;
        }


        if (hypot(vehicle_error.at(0), vehicle_error.at(1)) < 100) {
            desired_velocity = 0;
        }
        // // res.at(i) = desired_velocity;// * (double)vehicle_error.at(i) * 0.1; // 0.1: magic number
        // // これだけじゃダメ
        // int sign = i ? /* yについて */ (sin(running_direction) > 0) - (sin(running_direction) < 0) : /* xについて */ (cos(running_direction) > 0) - (cos(running_direction) < 0);
        // res.at(i) = sign * desired_velocity;
        res.at(i) = i ? fabs(vehicle_error.at(i)) * sin(running_direction) * 2.5 : fabs(vehicle_error.at(i)) * cos(running_direction) * 2.5;
    }
    res.at(2) = (0 - vehicle.current_configuration_[2]) * 3;
    return res;
}


#endif