/*
 * 老步兵云台板（LEGACY_INFANTRY_GIMBAL）在线状态收敛主机回归：
 * 0x065 ChassisBoard Daemon、RemoteInput 链路接口、DM-IMU 桥 freshness 与
 * Diagnostics 故障位映射。生产实现同步自老步兵云台分支当前源码。
 */
#include "daemon.h"
#include "board_config.h"
#include "chassis_board.h"
#include "Diagnostics.h"
#include "input_state.h"
#include "remote_input.h"
#include "message_center.h"
#include "sys_timestamp.h"
extern "C" {
#include "dvc_dm_imu.h"
#include "dm_imu_ins.h"
}
#include <vector>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)

uint32_t test_irq_mask;
uint64_t test_timestamp_us;
Class_Timestamp SYS_Timestamp;
FDCAN_HandleTypeDef hfdcan1{1}, hfdcan2{2}, hfdcan3{3};
struct Receiver { FDCAN_HandleTypeDef *bus; uint32_t id; CAN_RxCallback_t cb; void *ctx; };
static std::vector<Receiver> receivers;
static Struct_CAN_Tx_Msg sent;

extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void) { return test_timestamp_us; }
extern "C" uint32_t HAL_GetTick(void) { return test_timestamp_us / 1000; }
bool BSP_CAN_RegisterCallback(uint32_t id, FDCAN_HandleTypeDef *bus, CAN_RxCallback_t cb, void *ctx)
{ receivers.push_back({bus, id, cb, ctx}); return true; }
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *msg) { sent = *msg; return true; }
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *msg) { sent = *msg; return true; }

/* 生产 board_config.h 声明 BoardConfig_Get；板级配置由测试固定。 */
const BoardHardware &BoardConfig_Get(void)
{
    static BoardHardware board = {
        &hfdcan1, &hfdcan1, &hfdcan1, &hfdcan1, &hfdcan1,
        false, false, false, false, false, false,
        &hfdcan3, &hfdcan1, &hfdcan2};
    return board;
}

static void Time(uint64_t ms) { test_timestamp_us = ms * 1000; }

/** 0x065 合法 6 字节帧：fire=0x0102、dial=0x0304、pitch=0x0506（大端）。 */
static void FeedRemote(uint64_t ms, uint32_t len = 6)
{
    uint8_t frame[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0, 0};
    auto rx = receivers.front();
    Time(ms);
    rx.cb(rx.bus, rx.id, frame, len, rx.ctx);
}

static void ChassisLink()
{
    Class_ChassisBoard board;
    CHECK(board.Init(&hfdcan2));
    CHECK(!board.IsOnline());
    Struct_ChassisBoard_Channels ch{};
    CHECK(!board.ReadChannels(ch));                 // 从未收到数据。
    CHECK(!board.GetFire(nullptr));

    FeedRemote(0, 5);                               // 长度不足不 Feed。
    CHECK(!board.IsOnline() && !board.ReadChannels(ch));
    FeedRemote(0);
    CHECK(board.IsOnline());
    CHECK(board.ReadChannels(ch));
    CHECK(ch.fire == 0x0102 && ch.dial == 0x0304 && ch.pitch == 0x0506);
    CHECK(ch.timestamp_ms == 0);                    // 接收时间戳保留在快照中。

    FeedRemote(99);
    CHECK(board.IsOnline() && board.ReadChannels(ch));
    Time(199);                                      // 距上次合法帧 100 ms。
    CHECK(!board.IsOnline());                       // Daemon 门限（>=100 ms）先判离线。
    CHECK(board.ReadChannels(ch));                  // freshness 门限（>100 ms）独立判定。
    FeedRemote(200, 3);                             // 非法帧不能续期。
    CHECK(!board.IsOnline());
    CHECK(!board.ReadChannels(ch));                 // 101 ms：数据 freshness 也拒绝。
    FeedRemote(200);
    CHECK(board.IsOnline() && board.ReadChannels(ch));
    CHECK(ch.timestamp_ms == 200);
}

static void RemoteLink()
{
    CHECK(RemoteInput_Init());
    CHECK(!RemoteInput_IsLinkOnline());
    FeedRemote(0);
    CHECK(RemoteInput_IsLinkOnline());              // 直接返回 ChassisBoard Daemon 结果。
    RemoteInput_Update();
    const auto armed = InputState_Read();
    CHECK(armed.remote.valid);
    CHECK(armed.remote.shoot.shoot_mode == ShootMode::ON);
    CHECK(armed.remote.shoot.friction_mode == FrictionMode::OFF);

    Time(50);                                       // 100 ms 门限内：链路在线。
    RemoteInput_Update();
    CHECK(InputState_Read().remote.valid);
    Time(101);                                      // 链路离线：提交空输入，仲裁安全停。
    CHECK(!RemoteInput_IsLinkOnline());
    RemoteInput_Update();
    CHECK(!InputState_Read().remote.valid);
    CHECK(InputState_Read().remote.shoot.shoot_mode == ShootMode::OFF);
    FeedRemote(150);
    CHECK(RemoteInput_IsLinkOnline());
    RemoteInput_Update();
    CHECK(InputState_Read().remote.valid);
}

static void BridgeFreshness()
{
    CHECK(DM_IMU_InsBridge_Init());
    CHECK(!DM_IMU_InsBridge_IsFresh());
    Time(0);
    uint8_t euler[8] = {0x03, 0, 0, 0, 0, 0, 0, 0};
    auto rx = receivers.front();
    rx.cb(rx.bus, rx.id, euler, 8, rx.ctx);
    DM_IMU_InsBridge_Update();
    CHECK(MessageCenter::INS_State_Topic.Sequence() == 1);
    CHECK(DM_IMU_InsBridge_IsFresh());

    // Daemon 在线（未超时）但 Topic 数据对 10 ms 控制窗口已过期：
    // Online 与 ReadFresh 独立，旧姿态必须被控制层拒绝。
    Time(50);
    DM_IMU_InsBridge_Update();                      // 无新帧：不重复发布。
    CHECK(MessageCenter::INS_State_Topic.Sequence() == 1);
    INS_State ins{};
    CHECK(DM_IMU_IsOnline());
    CHECK(!MessageCenter::INS_State_Topic.ReadFresh(ins, 10000));

    Time(100);                                      // Daemon 先离线，桥 freshness 边界独立。
    CHECK(!DM_IMU_IsOnline());
    CHECK(DM_IMU_InsBridge_IsFresh());
    Time(101);
    CHECK(!DM_IMU_InsBridge_IsFresh());
    DM_IMU_InsBridge_Update();
    CHECK(MessageCenter::INS_State_Topic.Sequence() == 1);
    rx.cb(rx.bus, rx.id, euler, 8, rx.ctx);         // 新帧恢复发布。
    DM_IMU_InsBridge_Update();
    CHECK(MessageCenter::INS_State_Topic.Sequence() == 2);
    CHECK(DM_IMU_IsOnline());
}

static void DiagnosticsMap()
{
    Struct_Gimbal_Diagnostic g{};
    Struct_Shoot_Diagnostic s{};
    g.initialized = s.initialized = true;
    g.ins_valid = true;
    g.pitch = {true, true, false, true, true};
    s.left = {true, true, false, true, true};
    s.right = {true, true, false, true, true};
    s.loader = {true, true, false, true, true};

    CHECK(Diagnostics_BuildSnapshot(g, s, true, false).fault_mask == 0);

    // offline 位来自设备 Daemon 的 online 结果，不由诊断层自行计时。
    auto pitch_off = g; pitch_off.pitch.online = false;
    CHECK(Diagnostics_BuildSnapshot(pitch_off, s, true, false).fault_mask & DIAG_PITCH_OFFLINE);
    auto loader_off = s; loader_off.loader.online = false;
    CHECK(Diagnostics_BuildSnapshot(g, loader_off, true, false).fault_mask & DIAG_LOADER_OFFLINE);
    auto wheels_off = s; wheels_off.left.online = false; wheels_off.right.online = false;
    const auto wheel_bits = Diagnostics_BuildSnapshot(g, wheels_off, true, false).fault_mask;
    CHECK((wheel_bits & DIAG_LEFT_OFFLINE) && (wheel_bits & DIAG_RIGHT_OFFLINE));

    // required=false 的待命设备不报离线；fault / enable 超时仍是独立输入。
    auto idle = g; idle.pitch.required = false; idle.pitch.online = false;
    CHECK(!(Diagnostics_BuildSnapshot(idle, s, true, false).fault_mask & DIAG_PITCH_OFFLINE));
    CHECK(Diagnostics_BuildSnapshot(g, s, false, false).fault_mask & DIAG_REMOTE);
    auto stale_ins = g; stale_ins.ins_valid = false;
    CHECK(Diagnostics_BuildSnapshot(stale_ins, s, true, false).fault_mask & DIAG_INS);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!std::strcmp(argv[1], "chassis_link")) ChassisLink();
    else if (!std::strcmp(argv[1], "remote_link")) RemoteLink();
    else if (!std::strcmp(argv[1], "bridge_freshness")) BridgeFreshness();
    else if (!std::strcmp(argv[1], "diagnostics")) DiagnosticsMap();
    else CHECK(false);
    std::printf("PASS %s\n", argv[1]);
}
