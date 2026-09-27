#include "board_tasks.h"

void Board_CreateTasks(void)
{
    TransportTaskHandle = osThreadNew(Transport_Task, NULL, &TransportTask_attributes);
    CanTxTaskHandle = osThreadNew(Can_Tx_Task, NULL, &CanTxTask_attributes);
    StatusTaskHandle = osThreadNew(Status_Task, NULL, &StatusTask_attributes);
    TIM_1ms_TaskHandle = osThreadNew(TIM1msTask, NULL, &TIM_1ms_Task_attributes);
    /* BMI088 损坏的老步兵云台板不得创建会访问未初始化对象的任务；
     * 姿态由 FDCAN3 上的 DM-IMU 提供，在 Pitch 模块内轮询。 */
#if !LEGACY_INFANTRY_GIMBAL || LEGACY_INFANTRY_GIMBAL_YAW
    BMI088TaskHandle = osThreadNew(BMI088_Task, NULL, &BMI088Task_attributes);
#endif
    ControlTaskHandle = osThreadNew(Control_Task, NULL, &ControlTask_attributes);
    StorageTaskHandle = osThreadNew(Storage_Task, NULL, &StorageTask_attributes);
}
