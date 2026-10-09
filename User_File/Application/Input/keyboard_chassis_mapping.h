#ifndef KEYBOARD_CHASSIS_MAPPING_H
#define KEYBOARD_CHASSIS_MAPPING_H

#include "Chassis_Config.h"
#include "input_state.h"
#include "keyboard_logic.h"

inline ControlInput KeyboardChassis_Map(uint16_t keys, int16_t mouse_x, bool yaw_valid, float yaw_rad)
{
    const auto& c = kInfantryChassisConfig;
    const bool combo = KeyboardInput::Combination(keys);
    const bool shift = (keys & KeyboardInput::Shift) != 0U;
    const float ratio = shift ? c.keyboard_shift_ratio : c.keyboard_normal_ratio;
    float x = combo ? 0.0f : (int((keys & KeyboardInput::W) != 0U) - int((keys & KeyboardInput::S) != 0U)) * ratio * c.velocity_x_max;
    float y = combo ? 0.0f : (int((keys & KeyboardInput::A) != 0U) - int((keys & KeyboardInput::D) != 0U)) * ratio * c.velocity_y_max;
    const float error = yaw_valid ? std::remainder(yaw_rad - c.follow_forward_rad, 6.283185307179586f) : 0.0f;
    if (yaw_valid)
    {
        const float input_x = x;
        x = input_x * std::cos(error) - y * std::sin(error);
        y = input_x * std::sin(error) + y * std::cos(error);
    }
    else
    {
        // 没有相对 Yaw 时无法保证第一人称方向，保持平移为零，等待反馈。
        x = y = 0.0f;
    }
    const bool spin = !combo && (keys & (KeyboardInput::Q | KeyboardInput::E)) != 0U;
    ControlInput input{};
    float rotation = 0.0f;
    if (spin)
    {
        input.chassis.mode = ChassisMode::ROTATE;
        rotation = (int((keys & KeyboardInput::Q) != 0U) - int((keys & KeyboardInput::E) != 0U)) *
                   c.keyboard_spin_ratio * c.angular_velocity_max * (shift ? c.keyboard_spin_shift_multiplier : 1.0f);
    }
    else if (!combo && yaw_valid)
    {
        input.chassis.mode = ChassisMode::FOLLOW_GIMBAL_YAW;
        const float follow_error = std::copysign(std::fmax(std::fabs(error) - c.follow_deadband_rad, 0.0f), error);
        rotation = std::fmax(-c.follow_rotation_max, std::fmin(c.follow_rotation_max, -follow_error * c.follow_kp));
    }
    else
        input.chassis.mode = ChassisMode::NO_FOLLOW;
    input.chassis.velocity_x_m_s = std::fmax(-INPUT_MAX_TRANSLATION_M_S, std::fmin(INPUT_MAX_TRANSLATION_M_S, Chassis_TranslateX_ToSi(x)));
    input.chassis.velocity_y_m_s = std::fmax(-INPUT_MAX_TRANSLATION_M_S, std::fmin(INPUT_MAX_TRANSLATION_M_S, Chassis_TranslateY_ToSi(y)));
    input.chassis.angular_velocity_rad_s = Chassis_Rotation_ToSi(std::fmax(-c.angular_velocity_max, std::fmin(c.angular_velocity_max, rotation)));
    const int16_t bounded_x = mouse_x > c.keyboard_yaw_mouse_max ? c.keyboard_yaw_mouse_max : mouse_x < -c.keyboard_yaw_mouse_max ? -c.keyboard_yaw_mouse_max
                                                                                                                                  : mouse_x;
    input.gimbal.mode = GimbalMode::IMU;
    input.gimbal.yaw_speed_rad_s = -static_cast<float>(bounded_x) / c.keyboard_yaw_mouse_max * c.yaw_speed_max_rad_s;
    return input;
}

#endif
