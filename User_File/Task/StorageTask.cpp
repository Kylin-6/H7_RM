/**
 * @file StorageTask.cpp
 * @author zzm
 * @brief 底盘 Flash 故障记录；其他板型保留退出行为。
 */

/* Includes ------------------------------------------------------------------*/

#include "cmsis_os2.h"
#include "user_task.h"

#if CHASSIS
#include "../Application/Chassis/FlashLog/ChassisFlashLog.h"
#endif

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Storage_Task(void *argument)
{
    (void)argument;
#if CHASSIS
    ChassisFlashLog_Run();
#else
    osThreadExit();
#endif
}
