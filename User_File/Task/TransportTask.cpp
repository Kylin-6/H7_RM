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
/** JustFloat 发送周期，单位 ms。115200 波特率下 68 B 帧约占 5.9 ms，留足余量。 */
#define SHOOT_TELEMETRY_PERIOD_MS (10U)
/** JustFloat 通道数：3 路板间遥控通道 + 13 路发射诊断。 */
#define SHOOT_TELEMETRY_CHANNEL_COUNT (16U)
/** JustFloat 帧尾（IEEE754 约定的不可达浮点值）。 */
static const uint8_t SHOOT_TELEMETRY_TAIL[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

/*
 * 通道布局（与云台板原工程 TransportTask 的 JustFloat 顺序逐项一致）：
 *   [0] 火控开关    [1] 波轮档位    [2] Pitch 通道
 *   [3] 发射初始化  [4] 左轮反馈    [5] 右轮反馈    [6] 拨弹盘反馈
 *   [7] 摩擦轮就绪  [8] 左轮速度    [9] 右轮速度    [10] 左轮目标
 *   [11] 右轮目标   [12] 扳机状态   [13] 按下时长 ms
 *   [14] 左轮状态码 [15] 右轮状态码
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

            memcpy(justfloat_buffer, channels, sizeof(channels));
            memcpy(&justfloat_buffer[sizeof(channels)], SHOOT_TELEMETRY_TAIL,
                   sizeof(SHOOT_TELEMETRY_TAIL));
            /* USART1 未配置 TX DMA，走阻塞发送：68 B @115200 约 5.9 ms，
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
