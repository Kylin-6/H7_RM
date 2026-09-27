#include "Chassis.h"
#include "board_config.h"
#include "dji_motor.h"
#include "message_center.h"
#include <cstdio>
#include <cstdlib>

#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); std::exit(1); } } while (0)
uint32_t test_irq_mask;
uint64_t test_timestamp_us;
unsigned test_enable_count, test_disable_count, test_control_count;
static FDCAN_HandleTypeDef wheel{1}, steer{2};
const BoardHardware &BoardConfig_Get(void)
{
    static const BoardHardware hardware{&wheel, &steer};
    return hardware;
}
extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void) { return test_timestamp_us; }
int main()
{
    CHECK(Chassis_Init());
    CHECK(test_disable_count == 2);
    ChassisCmd command{};
    command.mode = ChassisMode::NO_FOLLOW;
    command.velocity_x_m_s = 1.0f;
    MessageCenter::Chassis_Command_Topic.PublishAt(command, 1000);
    test_timestamp_us = 101000;
    Chassis_Update();
    CHECK(test_enable_count == 2 && test_control_count == 2); // exact 100 ms remains fresh
    test_timestamp_us = 101001;
    Chassis_Update();
    CHECK(test_disable_count == 4 && test_control_count == 2); // stale command -> ZERO_FORCE
    std::puts("PASS chassis topic timeout to zero force");
}
