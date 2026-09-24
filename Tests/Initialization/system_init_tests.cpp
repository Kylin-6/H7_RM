#include "Init.h"
#include "system_devices.h"
#include <stdio.h>
#include <stdlib.h>

bool test_imu_ready, test_flash_ready;
unsigned test_fifo_starts, test_other_inits;
TestIMU BSP_BMI088;
TestFlash BSP_W25Q64JV;
TestDevice BSP_WS2812, BSP_Buzzer, BSP_Key, BSP_Power, EricTool_USB;
TestClock SYS_Timestamp;
int htim4, htim5, hspi2, hspi6, hospi2, hadc1;
int huart1, huart2, huart3, huart4, huart5, huart6, huart7, huart8, huart9, huart10;
extern volatile bool init_finished;

int main()
{
    // Begin with both failed and end with both healthy, proving errors are reset.
    for (int failures = 3; failures >= 0; --failures)
    {
        test_imu_ready = (failures & 1) == 0;
        test_flash_ready = (failures & 2) == 0;
        test_fifo_starts = test_other_inits = 0;
        System_Init();
        if (!init_finished || System_Get_Init_Errors() != (unsigned)failures ||
            test_fifo_starts != (unsigned)test_imu_ready || test_other_inits != 5)
        {
            fprintf(stderr, "FAIL startup with error mask %d\n", failures);
            return 1;
        }
        printf("PASS startup with error mask %d: returned, remaining devices initialized, FIFO gated\n", failures);
    }
    return 0;
}
