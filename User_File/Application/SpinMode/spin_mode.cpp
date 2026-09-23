#include "spin_mode.h"
#include "alg_matrix.h"

bool SpinMode::Init(const SpinConfig_t *config)
{
    const SpinConfig_t default_config = {0.0f, SpinMode_GIMBAL_FOLLOW};
    if (config == NULL)
    {
        config = &default_config;
    }
    if (Basic_Math_Is_Invalid_Float(config->zero_point) ||
        config->mode < SpinMode_GIMBAL_FOLLOW || config->mode > SpinMode_CHASSIS_FOLLOW)
    {
        return false;
    }

    zero_point = Basic_Math_Modulus_Normalization(config->zero_point, 2.0f * PI);
    world_angle = 0.0f;
    theta = zero_point;
    x_target = 0.0f;
    y_target = 0.0f;
    w_target = 0.0f;
    relative_target = 0.0f;
    capture_relative_target = true;
    Output = {};
    SpinMode_FSM.Init(config->mode);
    return true;
}

void SpinMode::Set_WorldTarget(float world_angle)
{
    this->world_angle = world_angle;
}

void SpinMode::Set_theta(float theta)
{
    this->theta = theta;
}

void SpinMode::Set_MoveTarget(float vx, float vy, float vw)
{
    x_target = vx;
    y_target = vy;
    w_target = vw;
}

void SpinMode::Set_Spin_Mode(SpinMode_e mode)
{
    if (mode < SpinMode_GIMBAL_FOLLOW || mode > SpinMode_CHASSIS_FOLLOW ||
        mode == Get_Spin_Mode())
    {
        return;
    }
    SpinMode_FSM.Set_Status(mode);
    capture_relative_target = true;
}

void SpinMode::TIM_Calculate_PeriodElapsedCallback()
{
    SpinMode_FSM.TIM_Calculate_PeriodElapsedCallback();
    const float relative_angle = Basic_Math_Modulus_Normalization(theta - zero_point, 2.0f * PI);
    Class_Matrix_f32<2, 1> velocity;
    velocity[0][0] = x_target;
    velocity[1][0] = y_target;
    velocity = Namespace_ALG_Matrix::From_Angle(relative_angle) * velocity;
    Output.x = velocity[0][0];
    Output.y = velocity[1][0];
    Output.w = w_target;
    Output.chassis_yaw_error = 0.0f;

    switch (Get_Spin_Mode())
    {
    case SpinMode_GIMBAL_FOLLOW:
        if (capture_relative_target)
        {
            relative_target = relative_angle;
            capture_relative_target = false;
        }
        Output.gimbal_yaw_target = relative_target;
        Output.forward = 0.0f;
        break;
    case SpinMode_CHASSIS_FOLLOW:
        Output.gimbal_yaw_target = world_angle;
        Output.chassis_yaw_error = relative_angle;
        Output.forward = -Output.w;
        break;
    case SpinMode_GIMBAL_LOCK:
        Output.gimbal_yaw_target = world_angle;
        Output.forward = -Output.w;
        break;
    }
}

SpinOutput_t SpinMode::Get_Output() const
{
    return Output;
}

SpinMode_e SpinMode::Get_Spin_Mode() const
{
    return (SpinMode_e)SpinMode_FSM.Get_Now_Status_Serial();
}

float SpinMode::Get_WorldTarget() const
{
    return world_angle;
}
