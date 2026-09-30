#include "daemon.h"
#include "dji_motor.h"
#include "sys_imu.h"
#include "bsp_bmi088.h"
#include "bsp_uart.h"
#include "sbus.h"
#include "remote_input.h"
#include "source_arbitration.h"
#include "message_center.h"
#include "sys_timestamp.h"
extern "C" {
#include "referee_26.h"
#include "vtm_26.h"
#include "crc_ref.h"
}
#include <array>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <limits>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)
uint32_t test_irq_mask;
uint64_t test_timestamp_us;
Class_Timestamp SYS_Timestamp;
FDCAN_HandleTypeDef hfdcan1{1}, hfdcan2{2}, hfdcan3{3};
UART_HandleTypeDef huart5;
TestBMI088 BSP_BMI088;
static DMA_HandleTypeDef dma;
static UART_Callback uart_callback;
struct Receiver { FDCAN_HandleTypeDef *bus; uint32_t id; CAN_RxCallback_t cb; void *ctx; };
static std::vector<Receiver> receivers;
static Struct_CAN_Tx_Msg sent;
extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void) { return test_timestamp_us; }
extern "C" uint32_t HAL_GetTick(void) { return test_timestamp_us / 1000; }
extern "C" void UART_Init(UART_HandleTypeDef *, UART_Callback cb) { uart_callback = cb; }
extern "C" HAL_StatusTypeDef UART_Transmit_Data(UART_HandleTypeDef *, uint8_t *, uint16_t) { return HAL_OK; }
extern "C" void osDelay(uint32_t) {}
bool BSP_CAN_RegisterCallback(uint32_t id, FDCAN_HandleTypeDef *bus, CAN_RxCallback_t cb, void *ctx)
{ receivers.push_back({bus, id, cb, ctx}); return true; }
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *msg) { sent = *msg; return true; }
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *) { return true; }
static void Time(uint64_t ms) { test_timestamp_us = ms * 1000; }
static unsigned offline_calls;
static void Offline(void *owner) { CHECK(owner == &offline_calls && test_irq_mask == 0); ++offline_calls; }
static void Transitions()
{
    Daemon monitor{20, Offline, &offline_calls};
    CHECK(!monitor.IsOnline() && monitor.Check() == DaemonTransition::None);
    CHECK(!monitor.SetTimeoutMs(0)); CHECK(monitor.SetTimeoutMs(30));
    test_irq_mask = 1; monitor.Feed(); CHECK(test_irq_mask == 1); test_irq_mask = 0;
    CHECK(monitor.IsOnline() && !monitor.SetTimeoutMs(40));
    CHECK(monitor.Check() == DaemonTransition::OfflineToOnline);
    CHECK(monitor.Check() == DaemonTransition::None);
    Time(29); CHECK(monitor.IsOnline());
    Time(35); CHECK(!monitor.IsOnline() && offline_calls == 0 && monitor.OfflineDurationMs() == 5);
    CHECK(monitor.Check() == DaemonTransition::OnlineToOffline && offline_calls == 1);
    Time(40); CHECK(monitor.Check() == DaemonTransition::None && offline_calls == 1);
    CHECK(monitor.OfflineDurationMs() == 10 && !monitor.SetTimeoutMs(50));
    monitor.Feed(); CHECK(monitor.IsOnline());
    CHECK(monitor.Check() == DaemonTransition::OfflineToOnline);
    CHECK(monitor.Check() == DaemonTransition::None);
    Time(70); monitor.Check(); CHECK(offline_calls == 2);
    Time(0xffffffffULL - 5); monitor.Feed(); monitor.Check();
    Time(0xffffffffULL + 24); CHECK(monitor.IsOnline());
    Time(0xffffffffULL + 25); CHECK(!monitor.IsOnline());
    CHECK(monitor.Check() == DaemonTransition::OnlineToOffline && offline_calls == 3);
}
static void FillRegistry()
{
    static Daemon monitors[] = {
        Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10},
        Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10},
        Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10},
        Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}, Daemon{10}};
    for (unsigned i = 0; i < 32; ++i) CHECK(DaemonManager::Register(monitors[i]));
    CHECK(DaemonManager::Register(monitors[0])); CHECK(!DaemonManager::Register(monitors[32]));
    monitors[31].Feed(); DaemonManager::CheckAll(); CHECK(monitors[31].IsOnline());
    Time(10); DaemonManager::CheckAll(); CHECK(!monitors[31].IsOnline());
}
static Struct_DJIMotor_Init_Config MotorConfig()
{
    Struct_DJIMotor_Init_Config c{};
    c.hfdcan = &hfdcan1; c.can_id = 1; c.motor_type = Enum_DJIMotor_Type::M3508;
    c.feedback_timeout_ms = 7; // 覆盖 Init 配置门限，而非固定默认值。
    return c;
}
static void DJI(bool registration_failure)
{
    if (registration_failure) FillRegistry();
    static Class_DJIMotor motor;
    CHECK(motor.Init(MotorConfig()) == !registration_failure);
    auto rx = receivers.back();
    uint8_t bytes[8] = {0, 1, 0, 0, 0, 0, 30, 0};
    if (registration_failure) {
        rx.cb(rx.bus, rx.id, bytes, 8, rx.ctx);
        CHECK(!motor.IsOnline() && !motor.GetDaemon().IsOnline()); return;
    }
    CHECK(!motor.IsOnline() && !motor.GetDaemon().IsOnline());
    rx.cb(&hfdcan2, rx.id, bytes, 8, rx.ctx);
    rx.cb(rx.bus, rx.id + 1, bytes, 8, rx.ctx);
    rx.cb(rx.bus, rx.id, bytes, 7, rx.ctx);
    bytes[0] = 0x20; rx.cb(rx.bus, rx.id, bytes, 8, rx.ctx);
    CHECK(!motor.IsOnline() && !motor.GetDaemon().IsOnline());
    bytes[0] = 0; rx.cb(rx.bus, rx.id, bytes, 8, rx.ctx);
    CHECK(motor.IsOnline() && motor.GetDaemon().IsOnline());
    static Class_DJIMotor_Group group;
    CHECK(group.Init(&motor)); CHECK(motor.RequestEnabled(true));
    CHECK(group.Control(123)); CHECK(sent.data[0] == 0 && sent.data[1] == 123);
    Time(6); bytes[0] = 0x20; rx.cb(rx.bus, rx.id, bytes, 8, rx.ctx);
    Time(8); // 不调用 StatusTask，Control/Snapshot 必须立即拒绝过期反馈并清零。
    CHECK(!motor.GetMotionSnapshot().ready && !motor.IsOnline());
    CHECK(!motor.GetDaemon().IsOnline()); CHECK(!group.Control(123));
    CHECK(sent.data[0] == 0 && sent.data[1] == 0);
    bytes[0] = 0; rx.cb(rx.bus, rx.id, bytes, 8, rx.ctx);
    CHECK(group.Control(123) && motor.IsHealthy());
}
static std::array<uint8_t,25> SbusFrame(uint8_t flags = 0)
{
    std::array<uint8_t,25> f{}; f[0]=0x0f; f[23]=flags;
    for (unsigned ch=0; ch<16; ++ch) { unsigned bit=ch*11+10; f[1+bit/8] |= 1U<<(bit%8); }
    return f;
}
static void SetupUart()
{ huart5.Init = {100000, UART_WORDLENGTH_9B, UART_STOPBITS_2, UART_PARITY_EVEN, UART_MODE_RX}; huart5.hdmarx = &dma; }
static void SbusInput()
{
    SetupUart(); CHECK(RemoteInput_Init()); CHECK(!SBUS_IsOnline());
    auto f=SbusFrame(); auto broken=f; broken[24]=0x7e;
    SBUS_RxCallback(broken.data(),25); CHECK(!SBUS_IsOnline());
    for (unsigned ms=0; ms<=200; ms+=20) {
        Time(ms); SBUS_RxCallback(f.data(),25); RemoteInput_Update();
        CHECK(InputState_Read().remote.valid == (ms==200));
    }
    CHECK(SourceArbitration_Resolve(InputState_Read()).armed);
    Time(251); RemoteInput_Update(); CHECK(SBUS_IsOnline());
    CHECK(!SourceArbitration_Resolve(InputState_Read()).armed);
    Time(252); auto fail=SbusFrame(0x0c); SBUS_RxCallback(fail.data(),25); RemoteInput_Update();
    CHECK(SBUS_IsOnline() && SBUS_IsDataValid() && !SBUS_IsHealthy());
    CHECK(!InputState_Read().remote.valid);
    for (unsigned ms=272; ms<=452; ms+=20) {
        Time(ms); SBUS_RxCallback(f.data(),25); RemoteInput_Update();
        CHECK(InputState_Read().remote.valid == (ms==452));
    }
    Time(552); CHECK(!SBUS_IsOnline());
}
static void INS()
{
    CHECK(System_IMU_Configure()); CHECK(!System_IMU_IsOnline());
    BSP_BMI088.initialized=false; System_IMU_Publish_State();
    CHECK(MessageCenter::INS_State_Topic.Sequence()==0);
    BSP_BMI088.initialized=true; System_IMU_Publish_State();
    CHECK(System_IMU_IsOnline() && MessageCenter::INS_State_Topic.Sequence()==1);
    INS_State state{}; test_timestamp_us=10000; CHECK(MessageCenter::INS_State_Topic.ReadFresh(state,10000));
    test_timestamp_us=10001; CHECK(!MessageCenter::INS_State_Topic.ReadFresh(state,10000));
    CHECK(System_IMU_IsOnline()); Time(30); CHECK(!System_IMU_IsOnline());
    for (unsigned i=0; i<6; ++i) {
        float *v=i<3 ? &BSP_BMI088.euler.Data[i] : &BSP_BMI088.gyro.Data[i-3];
        for (float invalid : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
            *v=invalid; System_IMU_Publish_State();
            CHECK(!System_IMU_IsOnline() && MessageCenter::INS_State_Topic.Sequence()==1);
        }
        *v=0;
    }
    Time(40); System_IMU_Publish_State(); CHECK(System_IMU_IsOnline());
    CHECK(MessageCenter::INS_State_Topic.Sequence()==2);
}
static std::vector<uint8_t> LinkFrame(uint16_t command=0xffff, unsigned payload=0)
{
    std::vector<uint8_t> f(9+payload); f[0]=0xa5; f[1]=payload; f[2]=payload>>8;
    f[5]=command; f[6]=command>>8; Append_CRC8_Check_Sum(f.data(),5);
    Append_CRC16_Check_Sum(f.data(),f.size()); return f;
}
static void Referee()
{
    auto *info=RefereeInit(&huart5); CHECK(info && !RefereeIsOnline());
    auto f=LinkFrame(); RefereeReceiveData(f.data(),4); CHECK(!RefereeIsOnline());
    RefereeReceiveData(f.data()+4,f.size()-4); CHECK(RefereeIsHealthy());
    Time(499); CHECK(RefereeIsOnline()); Time(500); CHECK(!RefereeIsOnline());
    auto corrupt=f; corrupt.back()^=1; RefereeReceiveData(corrupt.data(),corrupt.size()); CHECK(!RefereeIsOnline());
    corrupt=f; corrupt[4]^=1; RefereeReceiveData(corrupt.data(),corrupt.size()); CHECK(!RefereeIsOnline());
    auto wrong_length=LinkFrame(ID_game_result,2); RefereeReceiveData(wrong_length.data(),wrong_length.size());
    CHECK(!RefereeIsOnline()); RefereeReceiveData(f.data(),f.size()); CHECK(RefereeIsOnline());
    CHECK(info->CmdID==0xffff && info->GameResult.winner==0);
}
static void VTM()
{
    CHECK(VTMInit(&huart5)); CHECK(VTMIsEnabled() && !VTMIsOnline() && !VTMIsDataValid());
    std::array<uint8_t,21> rc{}; rc[0]=RC_SOF1; rc[1]=RC_SOF2;
    Append_CRC16_Check_Sum(rc.data(),rc.size());
    auto corrupt=rc; corrupt.back()^=1; VTMReceiveData(corrupt.data(),21); CHECK(!VTMIsOnline());
    VTMReceiveData(rc.data(),20); CHECK(!VTMIsOnline());
    VTMReceiveData(rc.data(),21); CHECK(VTMIsOnline() && VTMIsDataValid() && VTMIsHealthy());
    Time(299); CHECK(VTMIsOnline()); Time(300); CHECK(!VTMIsOnline());
    auto link=LinkFrame(); VTMReceiveData(link.data(),link.size()); CHECK(VTMIsHealthy());
    Time(600); CHECK(!VTMIsOnline());
    auto bad=link; bad[4]^=1; VTMReceiveData(bad.data(),bad.size()); CHECK(!VTMIsOnline());
    bad=link; bad.back()^=1; VTMReceiveData(bad.data(),bad.size()); CHECK(!VTMIsOnline());
    bad=link; bad[1]=0xff; bad[2]=0xff; Append_CRC8_Check_Sum(bad.data(),5);
    VTMReceiveData(bad.data(),bad.size()); CHECK(!VTMIsOnline());
}
static void Registration()
{
    FillRegistry(); SetupUart(); CHECK(!SBUS_Init(&huart5) && !SBUS_IsEnabled());
    CHECK(!System_IMU_Configure()); System_IMU_Publish_State();
    CHECK(!System_IMU_IsOnline() && MessageCenter::INS_State_Topic.Sequence()==0);
    CHECK(!RefereeInit(&huart5) && !RefereeIsEnabled());
    CHECK(!VTMInit(&huart5) && !VTMIsEnabled());
}
int main(int argc,char **argv)
{
    CHECK(argc==2);
    if (!std::strcmp(argv[1],"transitions")) Transitions();
    else if (!std::strcmp(argv[1],"capacity")) FillRegistry();
    else if (!std::strcmp(argv[1],"dji")) DJI(false);
    else if (!std::strcmp(argv[1],"dji_registration")) DJI(true);
    else if (!std::strcmp(argv[1],"sbus_input")) SbusInput();
    else if (!std::strcmp(argv[1],"ins")) INS();
    else if (!std::strcmp(argv[1],"referee")) Referee();
    else if (!std::strcmp(argv[1],"vtm")) VTM();
    else if (!std::strcmp(argv[1],"registration")) Registration();
    else CHECK(false);
    std::printf("PASS %s\n",argv[1]);
}
