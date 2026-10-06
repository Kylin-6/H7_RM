#ifndef CHASSIS_FLASH_LOG_H
#define CHASSIS_FLASH_LOG_H

#include <cstdint>

/** 固定小端格式；物理量为 SI，planned 为底盘原有抽象规划量。 */
struct Struct_Chassis_Flash_Record
{
    uint64_t timestamp_us;
    uint32_t sequence;
    uint32_t flags;
    float yaw[11];    ///< 与 Struct_Yaw_Tuning 前 11 个 float 同序。
    float command[3]; ///< vx m/s、vy m/s、w rad/s。
    float planned[3];
    float wheel_velocity[4]; ///< rad/s。
    float yaw_position_rad;
    uint32_t motor_states; ///< 四轮、Yaw 协议状态，各占 4 bit。
    uint32_t gimbal_age_us;
    uint32_t chassis_age_us;
    uint32_t fault_mask;
};
static_assert(sizeof(Struct_Chassis_Flash_Record) == 120U, "Flash record format");

/** ControlTask 单一生产者，仅复制 RAM，不执行 Flash I/O。 */
void ChassisFlashLog_Capture(const Struct_Chassis_Flash_Record& record);
/** StorageTask 唯一调用者，拥有 Flash 读写操作。 */
void ChassisFlashLog_Run();

struct Struct_Chassis_Flash_Debug
{
    uint32_t state; ///< 0 初始化、1 就绪、2 记录、3 满、4 I/O 错误、5 非日志数据、6 停止。
    uint32_t next_address;
    uint32_t written_records;
    uint32_t dropped_records;
    uint32_t command; ///< 调试器：1 停止并排空队列；2 读取（停止后）。
    uint32_t read_address;
    uint32_t read_length; ///< 1..4096 byte。
    uint32_t read_result; ///< 0 成功、1 I/O 错误、2 参数/状态不允许。
};
extern "C"
{
    extern volatile Struct_Chassis_Flash_Debug ChassisFlashLog_Debug;
    extern uint8_t ChassisFlashLog_ReadBuffer[4096];
}
#endif
