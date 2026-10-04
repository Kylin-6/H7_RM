#include "board_tasks.h"

void Board_CreateTasks(void)
{
    TransportTaskHandle = osThreadNew(Transport_Task, NULL, &TransportTask_attributes);
    CanTxTaskHandle = osThreadNew(Can_Tx_Task, NULL, &CanTxTask_attributes);
    StatusTaskHandle = osThreadNew(Status_Task, NULL, &StatusTask_attributes);
    TIM_1ms_TaskHandle = osThreadNew(TIM1msTask, NULL, &TIM_1ms_Task_attributes);
    /* BMI088 损坏的老步兵云台板不创建姿态任务；姿态由 FDCAN3 上的
     * DM-IMU 提供，在 DM-IMU 桥内轮询。 */
    (void)BMI088TaskHandle;
    ControlTaskHandle = osThreadNew(Control_Task, NULL, &ControlTask_attributes);
    StorageTaskHandle = osThreadNew(Storage_Task, NULL, &StorageTask_attributes);
}
