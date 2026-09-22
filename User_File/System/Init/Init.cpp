#include "Init.h"

#include "Com.h"
#include "SEGGER_SYSVIEW.h"
#include "bsp_adc.h"
#include "bsp_bmi088.h"
#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_key.h"
#include "bsp_ospi.h"
#include "bsp_power.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "bsp_w25q64jv.h"
#include "bsp_ws2812.h"
#include "callback.h"
#include "dvc_erictool.h"
#include "sys_debug.h"
#include "sys_imu.h"
#include "sys_timestamp.h"
#include "usart.h"

// 全局初始化完成标志位
volatile bool init_finished = false;
static volatile Enum_System_Init_State system_init_state = SYSTEM_INIT_READY;
static volatile uint32_t system_init_failure_mask = SYSTEM_INIT_FAILURE_NONE;

static void System_Init_RecordFailure(uint32_t failure,
                                      Enum_System_Init_State severity)
{
    system_init_failure_mask |= failure;
    if (severity > system_init_state)
    {
        system_init_state = severity;
    }
}

extern "C" Enum_System_Init_State System_Init_GetState(void)
{
    return system_init_state;
}

extern "C" uint32_t System_Init_GetFailureMask(void)
{
    return system_init_failure_mask;
}


extern "C" void System_Init(void)
{
    init_finished = false;
    system_init_state = SYSTEM_INIT_READY;
    system_init_failure_mask = SYSTEM_INIT_FAILURE_NONE;
    SEGGER_SYSVIEW_Conf();
    // SEGGER_SYSVIEW_Stop();
    SYS_Timestamp.Init(&htim5);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  

    UART_Init(&huart1, nullptr);
    UART_Init(&huart2, nullptr);
    UART_Init(&huart3, nullptr);
    UART_Init(&huart4, nullptr);
    UART_Init(&huart5, nullptr);
    UART_Init(&huart6, nullptr);
    UART_Init(&huart7, nullptr);
    UART_Init(&huart8, nullptr);
    UART_Init(&huart9, nullptr);
    UART_Init(&huart10, nullptr);

#if LEGACY_INFANTRY_GIMBAL && !LEGACY_INFANTRY_GIMBAL_YAW
    /*
     * 老步兵云台板：该板 BMI088 硬件故障，且默认不启用 Yaw 轴（没有姿态需求），
     * 因此不绑定 SPI2 回调，避免异常中断进入未初始化对象；与云台板原工程一致。
     * 启用 Yaw 轴（H7_LEGACY_INFANTRY_GIMBAL_YAW=ON）时按下方的 IMU 初始化走。
     */
    SPI_Init(&hspi2, nullptr);
#else
    // 陀螺仪的SPI
    SPI_Init(&hspi2, SPI2_Callback);
#endif

    // WS2812的SPI
    SPI_Init(&hspi6, nullptr);

    // Flash 的 OSPI
    OSPI_Init(&hospi2, OSPI2_Polling_Callback, OSPI2_Rx_Callback, OSPI2_Tx_Callback);

    if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
    {
        System_Init_RecordFailure(SYSTEM_INIT_FAILURE_TIM4, SYSTEM_INIT_FATAL);
    }
    if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
    {
        System_Init_RecordFailure(SYSTEM_INIT_FAILURE_TIM5, SYSTEM_INIT_FATAL);
    }
    if (system_init_state == SYSTEM_INIT_FATAL)
    {
        init_finished = true;
        return;
    }

#if LEGACY_INFANTRY_GIMBAL
    /*
     * ---- 老步兵云台板上电配置 ----
     *
     * 1. 与云台板原工程一致，只开启板载 5V；两路 24V 保持关闭，电机使用既有
     *    外部供电。未经硬件确认不得在移植层改变电源开关状态。
     * 2. 板上未安装 W25Q64JV：虽然框架侧 Init() 已带超时保护，但缺片时只会
     *    白白烧掉超时并记一次降级，因此与云台板原工程一致地跳过。
     *    OSPI 外设本身照常初始化，回调不受影响。
     * 3. BMI088 只在启用 Yaw 轴时初始化：云台板当前 IMU 硬件故障，Yaw 关闭时没有
     *    姿态需求；修好硬件后用 H7_LEGACY_INFANTRY_GIMBAL_YAW=ON 一并打开。
     * 4. ADC 与原工程一致不初始化，避免启用已知不可用的板载采样链路。
     */
    BSP_WS2812.Init();
    BSP_Buzzer.Init();
    BSP_Key.Init();
#if LEGACY_INFANTRY_GIMBAL_YAW
    System_IMU_Configure();
    if (!BSP_BMI088.Init())
    {
        System_Init_RecordFailure(SYSTEM_INIT_FAILURE_BMI088,
                                  SYSTEM_INIT_DEGRADED);
    }
#endif
    BSP_Power.Init(false, false, true);
    EricTool_USB.Init();
#if LEGACY_INFANTRY_GIMBAL_YAW
    BSP_BMI088.BMI088_Gyro.Start_FIFO_Acquisition();
#endif
#else
    System_IMU_Configure();
    const bool bmi088_initialized = BSP_BMI088.Init();
    if (!bmi088_initialized)
    {
        System_Init_RecordFailure(SYSTEM_INIT_FAILURE_BMI088,
                                  SYSTEM_INIT_DEGRADED);
    }
    BSP_WS2812.Init();
    BSP_Buzzer.Init();
    BSP_Key.Init();
    if (!BSP_W25Q64JV.Init())
    {
        System_Init_RecordFailure(SYSTEM_INIT_FAILURE_W25Q64,
                                  SYSTEM_INIT_DEGRADED);
    }
    if (!ADC_Init(&hadc1, 1))
    {
        System_Init_RecordFailure(SYSTEM_INIT_FAILURE_ADC1,
                                  SYSTEM_INIT_DEGRADED);
    }
    BSP_Power.Init(true, true, true);
    EricTool_USB.Init();
    if (bmi088_initialized)
    {
        BSP_BMI088.BMI088_Gyro.Start_FIFO_Acquisition();
    }
#endif
    
    init_finished = true;
}
