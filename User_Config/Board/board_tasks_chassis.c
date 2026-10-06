#include "board_tasks.h"

void Board_CreateTasks(void)
{
    /* 老步兵底盘板：SBUS 接收与 WS2812 刷新挂在 TIM_1ms_Task，姿态由 BMI088_Task
     * 发布；StorageTask 保存故障记录；不创建 USB 调试任务。 */
    CanTxTaskHandle = osThreadNew(Can_Tx_Task, NULL, &CanTxTask_attributes);
    StatusTaskHandle = osThreadNew(Status_Task, NULL, &StatusTask_attributes);
    TIM_1ms_TaskHandle = osThreadNew(TIM1msTask, NULL, &TIM_1ms_Task_attributes);
    BMI088TaskHandle = osThreadNew(BMI088_Task, NULL, &BMI088Task_attributes);
    StorageTaskHandle = osThreadNew(Storage_Task, NULL, &StorageTask_attributes);
    ControlTaskHandle = osThreadNew(Control_Task, NULL, &ControlTask_attributes);
}
