#include "sys_health.h"
#include "sys_debug.h"
#include "sys_timestamp.h"
#include "bsp_bmi088.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>

uint64_t test_timestamp_us;
uint64_t test_sample_us;
uint32_t test_irq_mask;
Class_Timestamp SYS_Timestamp;
Class_BMI088 BSP_BMI088;
extern "C" { volatile Sys_Debug_IMU_Data_t Debug_IMU_Data = {}; }
extern "C" void Status_Task(void *argument);

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static Struct_Sys_Health output;
static uint32_t tx_calls;
static CAN_RxCallback_t callbacks[32];
static void *contexts[32];
static FDCAN_HandleTypeDef *buses[32];
static uint32_t ids[32];
static uint32_t registered;
static Class_DJIMotor motors[25];
static FDCAN_HandleTypeDef handles[7];

bool BSP_CAN_RegisterCallback(uint32_t id, FDCAN_HandleTypeDef *bus, CAN_RxCallback_t callback, void *context)
{
    callbacks[registered] = callback;
    contexts[registered] = context;
    buses[registered] = bus;
    ids[registered++] = id;
    return true;
}
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *) { ++tx_calls; return true; }
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *) { ++tx_calls; return true; }

static void InitMotor(uint32_t index)
{
    Struct_DJIMotor_Init_Config config = {};
    config.hfdcan = &handles[index / 4];
    config.can_id = index % 4 + 1;
    config.motor_type = Enum_DJIMotor_Type::M3508;
    config.feedback_timeout_ms = 20;
    CHECK(motors[index].Init(config));
}

static void Feedback(uint32_t index)
{
    uint8_t bytes[8] = {1, 2, 0, 0, 0, 3, 25, 0};
    callbacks[index](buses[index], ids[index], bytes, 8, contexts[index]);
}

static void FreshIMU(uint64_t now)
{
    test_timestamp_us = now;
    test_sample_us = now;
    Debug_IMU_Data.Debug_ABI_Version = 5;
    Debug_IMU_Data.Debug_ABI_Size = sizeof(Debug_IMU_Data);
    Debug_IMU_Data.timestamp_us = now;
    Debug_IMU_Data.sequence += 2;
    Debug_IMU_Data.sequence_end = Debug_IMU_Data.sequence;
}

static void Update()
{
    Sys_Health_Update();
    CHECK(Sys_Health_GetSnapshot(&output));
    CHECK(output.Sequence == output.Sequence_End && (output.Sequence & 1) == 0);
}

static void Contract()
{
    CHECK(!Sys_Health_GetSnapshot(nullptr));
    CHECK(!Sys_Health_GetSnapshot(&output));
    Sys_Health_Init();
    CHECK(Sys_Health_GetSnapshot(&output) && output.Update_Count == 0);
    CHECK(output.Motor[0].Health.State == HEALTH_DISABLED);
    FreshIMU(1000);
    Update();
    CHECK(output.IMU.State == HEALTH_OK && output.Update_Count == 1);
    CHECK(output.ABI_Size == sizeof(output) && output.ABI_Version == 2);
    CHECK(output.Updated_At_us == 1000);
    test_irq_mask = 1;
    Update();
    CHECK(test_irq_mask == 1);
    test_irq_mask = 0;
    Update();
    CHECK(test_irq_mask == 0);
    CHECK(Debug_IMU_Data.sequence == 2 && Debug_IMU_Data.timestamp_us == 1000);
    SYS_Health.Sequence = 0xFFFFFFFE;
    SYS_Health.Sequence_End = 0xFFFFFFFE;
    Update();
    CHECK(output.Sequence == 0);
}

static void IMU()
{
    Sys_Health_Init();
    Update();
    CHECK(output.IMU.State == HEALTH_STARTING && output.IMU.Data_Age_ms == UINT32_MAX);
    FreshIMU(1000);
    test_sample_us = 0;
    Update();
    CHECK(output.IMU.State == HEALTH_STARTING);
    test_timestamp_us = 1000000;
    Update(); CHECK(output.IMU.State == HEALTH_STARTING);
    test_timestamp_us++;
    Update(); CHECK(output.IMU.State == HEALTH_OFFLINE && output.IMU.Reason == HEALTH_REASON_NO_DATA);
    FreshIMU(1100000);
    Update(); CHECK(output.IMU.State == HEALTH_OFFLINE && output.IMU.Reason == HEALTH_REASON_RECOVERING);
    Debug_IMU_Data.SPI2_Error_Counter++;
    Update(); CHECK(output.IMU.State == HEALTH_WARN && output.IMU.Reason == HEALTH_REASON_SPI);
    FreshIMU(1150000); Update(); CHECK(output.IMU.State == HEALTH_WARN);
    FreshIMU(1200000); Update(); CHECK(output.IMU.State == HEALTH_OK && output.IMU.Reason == 0);
    test_timestamp_us += 100000;
    Update(); CHECK(output.IMU.State == HEALTH_OK);
    test_timestamp_us++;
    Update(); CHECK(output.IMU.State == HEALTH_OFFLINE && output.IMU.Data_Age_ms == 100);
    FreshIMU(1400000); Update();
    FreshIMU(1450000); Update(); CHECK(output.IMU.State == HEALTH_OK);
    volatile uint32_t *counters[] = {
        &Debug_IMU_Data.SPI2_Error_Counter, &Debug_IMU_Data.BMI088_Transfer_Timeout_Counter,
        &Debug_IMU_Data.BMI088_SPI_Recovery_Counter, &Debug_IMU_Data.Gyro_Queue_Drop_Count,
        &Debug_IMU_Data.Gyro_FIFO_Overrun_Count, &Debug_IMU_Data.Gyro_Outlier_Counter,
        &Debug_IMU_Data.Accel_Update_Rejected_Counter, &Debug_IMU_Data.Timestamp_Anomaly_Counter,
        &Debug_IMU_Data.VQF_Reset_Counter,
    };
    const uint32_t reasons[] = {
        HEALTH_REASON_SPI, HEALTH_REASON_SPI, HEALTH_REASON_RECOVERY,
        HEALTH_REASON_SAMPLE_LOSS, HEALTH_REASON_SAMPLE_LOSS,
        HEALTH_REASON_INVALID_SAMPLE, HEALTH_REASON_INVALID_SAMPLE,
        HEALTH_REASON_TIMESTAMP, HEALTH_REASON_FILTER_RESET,
    };
    for (uint32_t i = 0; i < 9; ++i)
    {
        ++*counters[i];
        Update(); CHECK(output.IMU.State == HEALTH_WARN && output.IMU.Reason == reasons[i]);
        FreshIMU(test_timestamp_us + 50000); Update();
        CHECK(output.IMU.State == HEALTH_WARN && output.IMU.Reason == HEALTH_REASON_RECOVERING);
        FreshIMU(test_timestamp_us + 50000); Update();
        CHECK(output.IMU.State == HEALTH_OK);
    }
    Debug_IMU_Data.SPI2_Error_Counter = UINT32_MAX;
    Update();
    Debug_IMU_Data.SPI2_Error_Counter = 0;
    Update(); CHECK(output.IMU.Reason == HEALTH_REASON_SPI);
    FreshIMU(test_timestamp_us + 50000); test_sample_us++;
    Update(); CHECK(output.IMU.State == HEALTH_FAULT && (output.IMU.Reason & HEALTH_REASON_TIMESTAMP));
    FreshIMU(test_timestamp_us + 50000); Debug_IMU_Data.timestamp_us++;
    Update(); CHECK(output.IMU.State == HEALTH_FAULT);
    FreshIMU(test_timestamp_us + 50000); Update();
    FreshIMU(test_timestamp_us + 50000); Update(); CHECK(output.IMU.State == HEALTH_OK);
    test_timestamp_us += 150000;
    Debug_IMU_Data.timestamp_us = test_timestamp_us;
    Update(); CHECK(output.IMU.State == HEALTH_OFFLINE);
    Sys_Health_Init();
    FreshIMU(test_timestamp_us);
    Update(); CHECK(output.IMU.State == HEALTH_OK);
    Debug_IMU_Data.sequence = 0xFFFFFFFE;
    FreshIMU(test_timestamp_us + 50000);
    Update(); CHECK(Debug_IMU_Data.sequence == 0 && output.IMU.State == HEALTH_OK);
    test_timestamp_us += ((uint64_t)UINT32_MAX + 1) * 1000;
    Update(); CHECK(output.IMU.Data_Age_ms == UINT32_MAX && output.IMU.State == HEALTH_OFFLINE);
}

static void Motor()
{
    Sys_Health_Init();
    Struct_DJIMotor_Health snapshot = {};
    snapshot.rx_id = 0xDEAD;
    CHECK(!Class_DJIMotor::Get_Health(0, &snapshot) && snapshot.rx_id == 0xDEAD);
    CHECK(!Class_DJIMotor::Get_Health(24, &snapshot));
    CHECK(!Class_DJIMotor::Get_Health(0, nullptr));
    InitMotor(0);
    Update(); CHECK(output.Motor_Count == 1 && output.Motor[0].Health.State == HEALTH_STARTING);
    test_timestamp_us = 1000;
    Feedback(0);
    motors[0].current_pid.Set_Integral_Error(4);
    Update(); CHECK(output.Motor[0].Health.State == HEALTH_OK);
    CHECK(output.Motor[0].Bus == &handles[0] && output.Motor[0].Feedback_Id == 0x201);
    test_timestamp_us = 21000;
    Update(); CHECK(output.Motor[0].Health.State == HEALTH_OK);
    test_timestamp_us++;
    Update(); CHECK(output.Motor[0].Health.State == HEALTH_OFFLINE);
    CHECK(motors[0].online && motors[0].current_pid.Get_Integral_Error() == 4 && tx_calls == 0);
    Feedback(0); Update();
    CHECK(output.Motor[0].Health.State == HEALTH_OFFLINE && output.Motor[0].Health.Reason == HEALTH_REASON_RECOVERING);
    Feedback(0); Update(); CHECK(output.Motor[0].Health.State == HEALTH_OK);
    CHECK(motors[0].Disable());
    uint32_t previous_tx = tx_calls;
    Feedback(0); Update();
    CHECK(!output.Motor[0].Enabled && output.Motor[0].Health.State == HEALTH_OK);
    CHECK(tx_calls == previous_tx);
    test_irq_mask = 1;
    CHECK(Class_DJIMotor::Get_Health(0, &snapshot));
    CHECK(test_irq_mask == 1);
    test_irq_mask = 0;
    test_timestamp_us = 2000000;
    InitMotor(1); Update();
    CHECK(output.Motor[1].Health.State == HEALTH_STARTING);
    test_timestamp_us += 1000000;
    Update(); CHECK(output.Motor[1].Health.State == HEALTH_STARTING);
    test_timestamp_us++;
    Update(); CHECK(output.Motor[1].Health.State == HEALTH_OFFLINE);
}

static void Capacity()
{
    Sys_Health_Init();
    for (uint32_t i = 0; i < 24; ++i) InitMotor(i);
    FreshIMU(1000);
    for (uint32_t i = 0; i < 24; ++i) Feedback(i);
    Update();
    CHECK(output.Motor_Count == 24);
    for (uint32_t i = 0; i < 24; ++i)
    {
        CHECK(output.Motor[i].Health.State == HEALTH_OK);
        CHECK(output.Motor[i].Bus == &handles[i / 4]);
    }
    Struct_DJIMotor_Init_Config config = {};
    config.hfdcan = &handles[6]; config.can_id = 1;
    config.motor_type = Enum_DJIMotor_Type::M3508;
    CHECK(!motors[24].Init(config));
    CHECK(registered == 24 && tx_calls == 0);
}

static jmp_buf task_exit;
static uint32_t tick_rate, current_tick, tick_calls, work_ticks, delay_calls, deadlines[4];
static uint32_t wake_lateness_ticks;
uint32_t osKernelGetTickFreq() { return tick_rate; }
uint32_t osKernelGetTickCount()
{
    if (++tick_calls > 1) current_tick += work_ticks;
    return current_tick;
}
int32_t osDelayUntil(uint32_t ticks)
{
    CHECK((int32_t)(ticks - current_tick) > 0);
    deadlines[delay_calls++] = ticks;
    current_tick = ticks + wake_lateness_ticks;
    if (delay_calls == 4) longjmp(task_exit, 1);
    return 0;
}
static void Task()
{
    const uint32_t rates[3] = {100, 1000, 128};
    for (int trial = 0; trial < 9; ++trial)
    {
        tick_rate = rates[trial % 3];
        current_tick = 0xFFFFFFF0;
        tick_calls = delay_calls = 0;
        uint32_t period = (tick_rate * 50 + 999) / 1000;
        work_ticks = trial >= 3 && trial < 6 ? period + 2 : 0;
        wake_lateness_ticks = trial >= 6 ? period + 2 : 0;
        if (setjmp(task_exit) == 0) Status_Task(nullptr);
        for (uint32_t i = 1; i < 4; ++i)
            CHECK(deadlines[i] - deadlines[i - 1] == period + work_ticks + wake_lateness_ticks);
        CHECK(Sys_Health_GetSnapshot(&output) && output.Update_Count == 4);
    }
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    if (strcmp(argv[1], "contract") == 0) Contract();
    else if (strcmp(argv[1], "imu") == 0) IMU();
    else if (strcmp(argv[1], "motor") == 0) Motor();
    else if (strcmp(argv[1], "capacity") == 0) Capacity();
    else if (strcmp(argv[1], "task") == 0) Task();
    else return 2;
    printf("%s：%u 个检查点通过\n", argv[1], checks);
    return 0;
}