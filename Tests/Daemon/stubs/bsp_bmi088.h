#pragma once
#include "alg_matrix.h"
struct Struct_BMI088_VQF_Config {
    float Gyro_D_T, Accel_D_T;
    struct {
        float Tau_Accel, Bias_Sigma_Init_Deg_S, Bias_Forgetting_Time;
        float Bias_Clip_Deg_S, Bias_Sigma_Motion_Deg_S, Bias_Vertical_Forgetting_Factor;
        float Bias_Sigma_Rest_Deg_S, Rest_Min_Time, Rest_Filter_Tau;
        float Rest_Threshold_Gyro_Deg_S, Rest_Threshold_Accel;
        bool Motion_Bias_Estimation_Enable, Rest_Bias_Estimation_Enable;
    } Parameter;
};
struct TestBMI088 {
    bool initialized = true;
    Class_Matrix_f32<3, 1> euler, gyro;
    bool Is_Initialized() const { return initialized; }
    void Set_VQF_Config(const Struct_BMI088_VQF_Config &) {}
    Class_Matrix_f32<3, 1> Get_Euler_Angle() const { return euler; }
    Class_Matrix_f32<3, 1> Get_Gyro_Body() const { return gyro; }
};
extern TestBMI088 BSP_BMI088;
