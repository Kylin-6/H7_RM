#pragma once
#include "fdcan.h"
#include <stdint.h>
struct PID_InitTypeDef { float K_P, K_I, K_D, I_Out_Max, Out_Max, D_T; };
enum class Enum_DJIMotor_Type { M3508 };
enum Enum_DJIMotor_Loop : uint8_t { DJI_MOTOR_SPEED_LOOP = 2, DJI_MOTOR_ANGLE_LOOP = 4 };
struct Struct_DJIMotor_Init_Config {
    FDCAN_HandleTypeDef *hfdcan = nullptr;
    uint8_t can_id = 0;
    Enum_DJIMotor_Type motor_type = Enum_DJIMotor_Type::M3508;
    uint8_t close_loop = 0;
    Enum_DJIMotor_Loop outer_loop = DJI_MOTOR_SPEED_LOOP;
    PID_InitTypeDef speed_pid{}, angle_pid{};
};
struct TestMotorFeedback { float output_total_angle = 0, output_speed = 0; };
class Class_DJIMotor {
public:
    TestMotorFeedback feedback{};
    bool online = true;
    bool Init(const Struct_DJIMotor_Init_Config &) { return true; }
};
extern unsigned test_enable_count, test_disable_count, test_control_count;
class Class_DJIMotor_Group {
public:
    bool Init(Class_DJIMotor *, Class_DJIMotor *, Class_DJIMotor *, Class_DJIMotor *) { return true; }
    void Enable() { ++test_enable_count; }
    void Disable() { ++test_disable_count; }
    void Control(float, float, float, float) { ++test_control_count; }
};
