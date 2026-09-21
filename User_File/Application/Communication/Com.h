/**
 * @file Com.h
 * @brief 通信应用：外部输入适配与健康互锁。
 * @details
 * 默认配置下本模块只是一个空的 UART 帧入口骨架。
 *
 * 老步兵云台板配置（`LEGACY_INFANTRY_GIMBAL`）下，本模块承担底盘板到云台板的
 * 板间输入适配：
 *
 * ```text
 * 底盘板 0x065 (FDCAN2) -> Class_ChassisBoard -> Communication_Update()
 *        |- Pitch 通道 --两级低通 + 线性映射--> GimbalCmd.pitch_angle_rad
 *        |- 火控开关 -----> ShootCmd.shoot_mode（按下的电平）
 *        |- 波轮档位 --线性映射--> ShootCmd.loader_speed_deg_s
 * ```
 *
 * 链路超过 `CHASSIS_BOARD_CHANNEL_TIMEOUT_MS` 没有新帧时，本模块会显式发布
 * `GimbalMode::DISABLED` 与关闭的 `ShootCmd`，让云台与发射机构同时进入安全状态。
 */

#ifndef COM_H
#define COM_H

#include <stdbool.h>

#include "cmsis_os2.h"
#include "bsp_uart.h"

typedef struct
{
    uint8_t Header;
    uint16_t Length;
    uint8_t* Data;
    uint16_t Checksum;
} Frame_t;


void Communication_Callback(uint8_t* Buffer, uint16_t Length);

/**
 * @brief 初始化通信应用：老步兵云台板配置下注册板间链路接收。
 * @note 应在 RobotCmd 初始化之前调用一次。
 */
void Communication_Init(void);

/**
 * @brief 通信应用的 1 kHz 周期入口。
 * @note 先于 RobotCmd_Update 调用，保证本周期的命令来自本周期的输入。
 */
void Communication_Update(void);

/**
 * @brief 读取板间链路最近一帧的三个原始通道值（未滤波、未映射）。
 * @param fire  输出火控开关通道；可为 nullptr。
 * @param dial  输出拨弹盘（波轮）通道；可为 nullptr。
 * @param pitch 输出 Pitch 轴通道；可为 nullptr。
 * @return true 表示三个通道都有 100 ms 内的有效数据。
 * @note 只读访问器，不改变任何控制状态；供遥测任务观测链路原始量使用。
 *       非老步兵云台板配置下恒返回 false 并清零输出。
 */
bool Communication_GetRawChannels(int16_t *fire, int16_t *dial, int16_t *pitch);

#endif // COM_H
