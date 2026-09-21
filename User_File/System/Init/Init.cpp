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


extern "C" void System_Init(void)
{
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

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
#if LEGACY_INFANTRY_GIMBAL
    /*
     * ---- 老步兵云台板上电配置 ----
     *
     * 1. 两路 DC24 必须打开：云台 Pitch 电机、DM3519 摩擦轮与 M2006 拨弹盘的驱动器
     *    都由板载 24V 供电。云台板原工程写的是 BSP_Power.Init(false, false, true)
     *    （只开 5V、两路 24V 关闭），与"摩擦轮无反馈、达妙上位机也读不到电机"的
     *    现象吻合；若实车确认电机改由外部供电，把前两个参数改回 false 即可。
     * 2. 板上未使用 W25Q64JV：其 Init() 会在等待 JEDEC ID 处一直自旋
     *    （while (Rx_Buffer != 0x001740EF)），缺片或坏片会把 System_Init 卡死，
     *    因此与云台板原工程一致地跳过。OSPI 外设本身照常初始化，回调不受影响。
     * 3. BMI088 只在启用 Yaw 轴时初始化：云台板当前 IMU 硬件故障，Yaw 关闭时没有
     *    姿态需求；修好硬件后用 H7_LEGACY_INFANTRY_GIMBAL_YAW=ON 一并打开。
     * 4. ADC 保留初始化：BSP_Power 绑定 ADC1_Manage_Object，跳过会让电源电压读取
     *    指向未初始化的缓冲区；本外设无副作用。
     */
    BSP_WS2812.Init();
    BSP_Buzzer.Init();
    BSP_Key.Init();
#if LEGACY_INFANTRY_GIMBAL_YAW
    System_IMU_Configure();
    BSP_BMI088.Init();
#endif
    ADC_Init(&hadc1, 1);
    BSP_Power.Init(true, true, true);
    EricTool_USB.Init();
#if LEGACY_INFANTRY_GIMBAL_YAW
    BSP_BMI088.BMI088_Gyro.Start_FIFO_Acquisition();
#endif
#else
    System_IMU_Configure();
    BSP_BMI088.Init();
    BSP_WS2812.Init();
    BSP_Buzzer.Init();
    BSP_Key.Init();
    BSP_W25Q64JV.Init();
    ADC_Init(&hadc1, 1);
    BSP_Power.Init(true, true, true);
    EricTool_USB.Init();
    BSP_BMI088.BMI088_Gyro.Start_FIFO_Acquisition();
#endif
    
    init_finished = true;
}
