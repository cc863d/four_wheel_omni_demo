#ifndef VELOCITY_CONTROLLER
#define VELOCITY_CONTROLLER

#include "four_wheel_omni.hpp"
#include "PID.h"

class VelocityController
{
private:
    FourWheelOmni *vehicle_;
    PID *pid_;

public:
    VelocityController(FourWheelOmni *vehicle, PID *pid);
    ~VelocityController();
    void SetDesiredValue(double desired_velue[3]);
    void SetDesiredValue(std::array<double, 3> desired_velue);
    void SetDesiredValue(std::array<double, 3> desired_velue, bool stop_condition);
    void SetControlParam(double control_param[3]);

    Thread execute_one_cycle_thread_;
    void ExecuteOneCycleThreadDriver();
    void ExecuteOneCycle(void);
};

VelocityController::VelocityController(FourWheelOmni *vehicle, PID *pid)
{
    vehicle_ = vehicle;
    pid_ = pid;
    execute_one_cycle_thread_.start(callback(this, &VelocityController::ExecuteOneCycle));
}

VelocityController::~VelocityController()
{
}

void VelocityController::SetDesiredValue(double desired_velue[3])
{
    for (int i = 0; i < 3; i++)
    {
        vehicle_->desired_velocity_[i] = desired_velue[i];
    }
}

void VelocityController::SetDesiredValue(std::array<double, 3> desired_velue)
{
    for (int i = 0; i < 3; i++)
    {
        vehicle_->desired_velocity_[i] = desired_velue[i];
    }
}

void VelocityController::SetDesiredValue(std::array<double, 3> desired_velue, bool stop_condition)
{
    if (stop_condition == true)
    {
        for (int i = 0; i < 3; i++)
        {
            vehicle_->desired_velocity_[i] = 0;
        }
        // vehicle_->DisableControl();
    }
    else
    {
        SetDesiredValue(desired_velue);
    }
}

void VelocityController::SetControlParam(double control_param[3])
{
    pid_->setTunings(control_param[0], control_param[1], control_param[2]);
}

void VelocityController::ExecuteOneCycle()
{
    while (ThisThread::flags_wait_all(1, true))
    {
        // [issue]  （今は制御状態を考えていないが）
        //          制御状態がオフのとき，どういう挙動を定義すればよい？
        double set_velocity[3];           // 指示しようとしている動き x'[mm/s] , y'[mm/s], theta'[rad/s]
        double set_velocities[4];         // 各車輪の周速度
        double set_angular_velocities[4]; // 各車輪の角速度

        for (int i = 0; i < 3; i++)
        {
            set_velocity[i] = vehicle_->desired_velocity_[i];
        }

        // 並進移動しかしないことにして
        // 掛け算はマジックナンバー
        set_velocity[2] = (0 - vehicle_->current_configuration_[2]) / 200;

        set_velocities[0] = (set_velocity[0] - set_velocity[1] + set_velocity[2] * vehicle_->wheel_separation_width_);
        set_velocities[1] = (set_velocity[0] + set_velocity[1] + set_velocity[2] * vehicle_->wheel_separation_width_);
        set_velocities[2] = (-set_velocity[0] + set_velocity[1] + set_velocity[2] * vehicle_->wheel_separation_width_);
        set_velocities[3] = (-set_velocity[0] - set_velocity[1] + set_velocity[2] * vehicle_->wheel_separation_width_);

        for (int i = 0; i < 4; i++)
        {
            set_angular_velocities[i] = set_velocities[i] / (vehicle_->wheel_radius_);
        }

        vehicle_->SetDesiredAngularVelocities(set_angular_velocities);
    }
}
void VelocityController::ExecuteOneCycleThreadDriver()
{
    execute_one_cycle_thread_.flags_set(1);
}

#endif