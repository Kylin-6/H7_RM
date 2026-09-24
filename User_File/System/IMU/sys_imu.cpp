/**
 * @file sys_imu.cpp
 * @author zzm
 * @brief IMU系统级参数配置与状态发布
 * @version 1.0
 * @date 2026-08-12
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_imu.h"

#include "bsp_bmi088.h"
#include "bsp_uart.h"
#include "message_center.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static Publisher<INS_State> INS_State_Publisher(MessageCenter::INS_State_Topic);

static constexpr uint16_t WIT_FRAME_LENGTH = 11U;
static constexpr uint64_t WIT_MAX_AGE_US = 120000U;
static constexpr float WIT_ANGLE_SCALE = 3.14159265358979323846f / 32768.0f;
static constexpr float WIT_RATE_SCALE = 2000.0f * WIT_ANGLE_SCALE / 180.0f;

struct Wit_State
{
    INS_State ins;
    uint64_t angle_timestamp_us = 0U;
    uint64_t rate_timestamp_us = 0U;
};

static Wit_State Wit_Latest_State;
static uint8_t Wit_Frame[WIT_FRAME_LENGTH];
static uint8_t Wit_Frame_Index;
static bool Wit_Yaw_Zero_Valid;
static float Wit_Yaw_Zero;
static bool Wit_Fallback_Enabled;

static int16_t Wit_Read_Int16(const uint8_t *bytes)
{
    return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) |
                                (static_cast<uint16_t>(bytes[1]) << 8U));
}

static void Wit_UART_Callback(uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0U; i < length; ++i)
    {
        const uint8_t byte = buffer[i];
        if (Wit_Frame_Index == 0U && byte != 0x55U)
        {
            continue;
        }
        Wit_Frame[Wit_Frame_Index++] = byte;
        if (Wit_Frame_Index == 2U &&
            Wit_Frame[1] != 0x52U && Wit_Frame[1] != 0x53U)
        {
            Wit_Frame_Index = byte == 0x55U ? 1U : 0U;
            continue;
        }
        if (Wit_Frame_Index != WIT_FRAME_LENGTH)
        {
            continue;
        }
        Wit_Frame_Index = 0U;
        uint8_t checksum = 0U;
        for (uint8_t j = 0U; j < WIT_FRAME_LENGTH - 1U; ++j)
        {
            checksum = static_cast<uint8_t>(checksum + Wit_Frame[j]);
        }
        if (checksum != Wit_Frame[WIT_FRAME_LENGTH - 1U])
        {
            continue;
        }

        const uint64_t timestamp_us = SYS_Timestamp_Get_Microsecond();
        if (Wit_Frame[1] == 0x52U)
        {
            Wit_Latest_State.ins.gyro_x_rad_s = Wit_Read_Int16(&Wit_Frame[2]) * WIT_RATE_SCALE;
            Wit_Latest_State.ins.gyro_y_rad_s = Wit_Read_Int16(&Wit_Frame[4]) * WIT_RATE_SCALE;
            Wit_Latest_State.ins.gyro_z_rad_s = Wit_Read_Int16(&Wit_Frame[6]) * WIT_RATE_SCALE;
            Wit_Latest_State.rate_timestamp_us = timestamp_us;
        }
        else
        {
            Wit_Latest_State.ins.roll_rad = Wit_Read_Int16(&Wit_Frame[2]) * WIT_ANGLE_SCALE;
            Wit_Latest_State.ins.pitch_rad = Wit_Read_Int16(&Wit_Frame[4]) * WIT_ANGLE_SCALE;
            const float yaw = Wit_Read_Int16(&Wit_Frame[6]) * WIT_ANGLE_SCALE;
            if (!Wit_Yaw_Zero_Valid)
            {
                Wit_Yaw_Zero = yaw;
                Wit_Yaw_Zero_Valid = true;
            }
            float relative_yaw = yaw - Wit_Yaw_Zero;
            if (relative_yaw > 3.14159265358979323846f)
            {
                relative_yaw -= 6.28318530717958647692f;
            }
            else if (relative_yaw < -3.14159265358979323846f)
            {
                relative_yaw += 6.28318530717958647692f;
            }
            Wit_Latest_State.ins.yaw_rad = relative_yaw;
            Wit_Latest_State.angle_timestamp_us = timestamp_us;
        }
    }
}

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 设置BMI088使用的VQF姿态与零偏估计参数
 *
 * @details
 * 此处只选择整机参数并交给BMI088，不包含传感器操作或算法实现。
 * 参数在BSP_BMI088.Init()中用于初始化VQF，因此必须先配置、后初始化。
 */
void System_IMU_Configure()
{
    Struct_BMI088_VQF_Config config;

    // VQF更新周期。陀螺仪按2 kHz逐帧积分，加速度计按250 Hz修正重力方向。
    config.Gyro_D_T = 0.0005f;
    config.Accel_D_T = 0.004f;

    // 加速度修正时间常数越大，Pitch/Roll越平滑，但收敛到重力方向越慢。
    config.Parameter.Tau_Accel = 3.0f;

    // 运动和静止零偏估计同时启用：运动时缓慢跟踪，静止确认后提高可信度。
    config.Parameter.Motion_Bias_Estimation_Enable = true;
    config.Parameter.Rest_Bias_Estimation_Enable = true;

    // 初始零偏不确定度。数值越大，启动阶段允许零偏估计更快调整。
    config.Parameter.Bias_Sigma_Init_Deg_S = 0.5f;

    // 零偏遗忘时间越长，长期估计越稳定，但温漂变化后的重新收敛越慢。
    config.Parameter.Bias_Forgetting_Time = 100.0f;

    // 限制零偏估计和残差的最大幅度，避免真实转动被误当作零偏。
    config.Parameter.Bias_Clip_Deg_S = 2.0f;

    // 运动状态下的零偏观测噪声；越小越信任运动过程中的估计。
    config.Parameter.Bias_Sigma_Motion_Deg_S = 0.1f;

    // 降低运动时重力轴方向零偏的观测权重，越小越保守。
    config.Parameter.Bias_Vertical_Forgetting_Factor = 0.0001f;

    // 静止状态下的零偏观测噪声；小于运动值，使静止零偏更新更可信。
    config.Parameter.Bias_Sigma_Rest_Deg_S = 0.03f;

    // 陀螺仪和加速度计连续满足静止条件1.5 s后，才进入静止零偏估计。
    config.Parameter.Rest_Min_Time = 1.5f;

    // 静止检测低通时间常数，抑制瞬时振动导致的状态反复切换。
    config.Parameter.Rest_Filter_Tau = 0.5f;

    // 静止门限：角速度残差不超过3.5 deg/s，加速度残差不超过0.5 m/s^2。
    config.Parameter.Rest_Threshold_Gyro_Deg_S = 3.5f;
    config.Parameter.Rest_Threshold_Accel = 0.5f;

    BSP_BMI088.Set_VQF_Config(config);
}

/**
 * @brief 将 BMI088/VQF 最新输出转换为与传感器无关的 INS 状态。
 * @details
 * 此函数位于原有 FIFO、SPI DMA 和姿态解算链路之后，只负责整理并发布结果；
 * Application 因而只依赖 INS_State，不直接依赖 BMI088 设备对象。
 */
void System_IMU_Publish_State()
{
    if (!BSP_BMI088.Is_Initialized())
    {
        return;
    }
    const Class_Matrix_f32<3, 1> euler = BSP_BMI088.Get_Euler_Angle();
    const Class_Matrix_f32<3, 1> gyro_body = BSP_BMI088.Get_Gyro_Body();
    const INS_State ins_state = {
        .yaw_rad = euler.Data[0],
        .pitch_rad = euler.Data[1],
        .roll_rad = euler.Data[2],
        .gyro_x_rad_s = gyro_body.Data[0],
        .gyro_y_rad_s = gyro_body.Data[1],
        .gyro_z_rad_s = gyro_body.Data[2],
    };
    /* 高频姿态使用静态 Topic，避免动态队列进入 1 kHz 闭环路径。 */
    INS_State_Publisher.Publish(ins_state);
}

void System_IMU_Start_Wit_Fallback()
{
    Wit_Latest_State = {};
    Wit_Frame_Index = 0U;
    Wit_Yaw_Zero_Valid = false;
    Wit_Fallback_Enabled = false;

    uint8_t wake[5] = {0xFFU, 0xAAU, 0x69U, 0x88U, 0xB5U};
    (void)HAL_UART_Transmit(&huart7, wake, sizeof(wake), 20U);
    uint8_t reset[5] = {0xFFU, 0xAAU, 0x00U, 0x01U, 0x00U};
    uint8_t save[5] = {0xFFU, 0xAAU, 0x00U, 0x00U, 0x00U};
    HAL_Delay(200U);
    (void)HAL_UART_Transmit(&huart7, reset, sizeof(reset), 20U);
    HAL_Delay(200U);
    (void)HAL_UART_Transmit(&huart7, save, sizeof(save), 20U);

    UART_Init(&huart7, Wit_UART_Callback);
    // UART BSP 会在 DMA 启动失败后重试，保持回退发布路径可用。
    Wit_Fallback_Enabled = true;
}

void System_IMU_Publish_Wit_Fallback()
{
    if (!Wit_Fallback_Enabled)
    {
        return;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const Wit_State snapshot = Wit_Latest_State;
    __set_PRIMASK(primask);

    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    if (snapshot.rate_timestamp_us == 0U || now_us < snapshot.rate_timestamp_us ||
        now_us - snapshot.rate_timestamp_us > WIT_MAX_AGE_US)
    {
        return;
    }
#if !LEGACY_INFANTRY
    if (snapshot.angle_timestamp_us == 0U || now_us < snapshot.angle_timestamp_us ||
        now_us - snapshot.angle_timestamp_us > WIT_MAX_AGE_US)
    {
        return;
    }
#endif
    INS_State_Publisher.Publish(snapshot.ins);
}
