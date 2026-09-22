/**
 * @file    TransportTask.cpp
 * @brief   传输任务 —— USB CDC 初始化与遥测输出
 * @author  zzm
 * @version 1.3
 * @date    2026-09-21 1.3 老步兵云台板配置下恢复 USART1 JustFloat 发射遥测
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"
#include "usb_device.h"
#include "user_task.h"

#if LEGACY_INFANTRY_GIMBAL
#include "Com.h"
#include "Shoot.h"
#include "bsp_uart.h"
#include "usart.h"

#include <string.h>
#endif


/* Private macros ------------------------------------------------------------*/

#if LEGACY_INFANTRY_GIMBAL
/** JustFloat 发送周期，单位 ms。115200 波特率下 100 B 帧约占 8.7 ms。 */
#define SHOOT_TELEMETRY_PERIOD_MS (10U)
/** JustFloat 通道数：原 16 路诊断 + 8 路 M2006/C610 排查数据。 */
#define SHOOT_TELEMETRY_CHANNEL_COUNT (24U)
/** JustFloat 帧尾（IEEE754 约定的不可达浮点值）。 */
static const uint8_t SHOOT_TELEMETRY_TAIL[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

/*
 * 通道布局（与云台板原工程 TransportTask 的 JustFloat 顺序逐项一致）：
 *   [0] 火控开关    [1] 波轮档位    [2] Pitch 通道
 *   [3] 发射初始化  [4] 左轮反馈    [5] 右轮反馈    [6] 拨弹盘反馈
 *   [7] 摩擦轮就绪  [8] 左轮速度    [9] 右轮速度    [10] 左轮目标
 *   [11] 右轮目标   [12] 扳机状态   [13] 按下时长 ms
 *   [14] 左轮状态码 [15] 右轮状态码
 *   [16] M2006编码器 [17] 转子累计角度° [18] 输出轴累计角度°
 *   [19] 转子速度rad/s [20] 输出轴速度rad/s [21] 原始电流
 *   [22] 反馈年龄ms（从未收到为-1） [23] 速度PID输出
 */
#endif

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *argument)
{
    MX_USB_DEVICE_Init();
    EricTool_USB.Set_Data(3, (int) &Debug_IMU_Data.Euler_Yaw_rad,
                         (int) &Debug_IMU_Data.Euler_Pitch_rad,
                         (int) &Debug_IMU_Data.Euler_Roll_rad);
#if LEGACY_INFANTRY_GIMBAL
    uint32_t uart_last_wake_time = osKernelGetTickCount();
#endif
    for (;;)
    {
        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
#if LEGACY_INFANTRY_GIMBAL
        /* 云台板原工程用 USART1 JustFloat 观察板间通道与发射诊断，这里保持 100 Hz。 */
        {
            // 静态缓冲避免占用线程栈；小端序下 float 数组可直接按字节发送。
            static uint8_t justfloat_buffer[SHOOT_TELEMETRY_CHANNEL_COUNT * 4U +
                                            sizeof(SHOOT_TELEMETRY_TAIL)];
            float channels[SHOOT_TELEMETRY_CHANNEL_COUNT];

            int16_t fire = 0;
            int16_t dial = 0;
            int16_t pitch = 0;
            const bool channels_valid = Communication_GetRawChannels(&fire, &dial, &pitch);
            channels[0] = channels_valid ? static_cast<float>(fire) : 0.0f;
            channels[1] = channels_valid ? static_cast<float>(dial) : 0.0f;
            channels[2] = channels_valid ? static_cast<float>(pitch) : 0.0f;
            Shoot_GetDebug(&channels[3], &channels[4], &channels[5],
                           &channels[6], &channels[7], &channels[8],
                           &channels[9], &channels[10], &channels[11],
                           &channels[12], &channels[13], &channels[14],
                           &channels[15]);
            Struct_Legacy_Loader_Debug loader_debug{};
            Shoot_GetLoaderDebug(&loader_debug);
            channels[16] = loader_debug.encoder;
            channels[17] = loader_debug.rotor_total_angle_degree;
            channels[18] = loader_debug.output_total_angle_degree;
            channels[19] = loader_debug.rotor_speed_rad_s;
            channels[20] = loader_debug.output_speed_rad_s;
            channels[21] = loader_debug.current_raw;
            channels[22] = loader_debug.feedback_age_ms;
            channels[23] = loader_debug.speed_pid_out;

            memcpy(justfloat_buffer, channels, sizeof(channels));
            memcpy(&justfloat_buffer[sizeof(channels)], SHOOT_TELEMETRY_TAIL,
                   sizeof(SHOOT_TELEMETRY_TAIL));
            /* USART1 未配置 TX DMA，走阻塞发送：100 B @115200 约 8.7 ms，
             * 在 10 ms 超时内完成；本任务优先级 Normal，不影响 1 kHz 控制任务。 */
            (void) UART_Transmit_Data(&huart1, justfloat_buffer,
                                      sizeof(justfloat_buffer));

            uart_last_wake_time += SHOOT_TELEMETRY_PERIOD_MS;
            osDelayUntil(uart_last_wake_time);
        }
#else
        osDelay(1);
#endif
    }
}
