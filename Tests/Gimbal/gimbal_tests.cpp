#include "Gimbal.h"
#include "dmmotor.h"
#include "message_center.h"
#include "sys_timestamp.h"
#include "board_config.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)
FDCAN_HandleTypeDef hfdcan1{1}, hfdcan2{2}, hfdcan3{3};
const BoardHardware &BoardConfig_Get(void)
{
    static const BoardHardware hardware{&hfdcan1, &hfdcan1, nullptr, nullptr,
                                         nullptr, false, false, false, false, false, false};
    return hardware;
}
uint32_t test_irq_mask;
uint64_t test_timestamp_us;
Class_Timestamp SYS_Timestamp;
extern "C" uint64_t SYS_Timestamp_Get_Microsecond() { return test_timestamp_us; }
struct Receiver { uint32_t id; FDCAN_HandleTypeDef *bus; CAN_RxCallback_t fn; void *context; };
static std::vector<Receiver> receivers;
static std::vector<Struct_CAN_Tx_Msg> discrete, periodic;
static bool submit_ok = true, perform_ok = true, registration_ok = true;
static unsigned submit_attempts, perform_attempts;
static std::vector<char> operations;
static unsigned registration_limit = 2;
static Struct_Gimbal_Config cfg;
static INS_State ins{};
static GimbalCmd command{};
static float motor_position = -0.2f, motor_velocity = 0.0f;

bool BSP_CAN_RegisterCallback(uint32_t id, FDCAN_HandleTypeDef *bus, CAN_RxCallback_t fn, void *context)
{
    if (!registration_ok || receivers.size() >= registration_limit) return false;
    receivers.push_back({id, bus, fn, context});
    return true;
}
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *message)
{
    ++submit_attempts;
    operations.push_back('D');
    if (submit_ok) discrete.push_back(*message);
    return submit_ok;
}
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *message)
{
    ++perform_attempts;
    operations.push_back('P');
    if (perform_ok) periodic.push_back(*message);
    return perform_ok;
}
static bool Near(float a, float b, float tolerance = 0.02f) { return std::fabs(a - b) < tolerance; }
static uint16_t Encode(float value, float maximum, unsigned bits)
{
    return static_cast<uint16_t>((value + maximum) * ((1U << bits) - 1U) / (2 * maximum));
}
static void Receive(const Struct_Gimbal_Motor_Config &c, int state, float position, float velocity)
{
    const uint16_t p = Encode(position, c.position_max, 16);
    const uint16_t v = Encode(velocity, c.velocity_max, 12);
    const uint16_t t = Encode(0, c.torque_max, 12);
    uint8_t frame[8] = {static_cast<uint8_t>((state << 4) | (c.id & 15)),
        static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p), static_cast<uint8_t>(v >> 4),
        static_cast<uint8_t>((v << 4) | (t >> 8)), static_cast<uint8_t>(t), 25, 26};
    for (const auto &r : receivers)
        if (r.id == c.feedback_id && r.bus == c.bus) r.fn(c.bus, c.feedback_id, frame, 8, r.context);
}
static void Tick(uint64_t elapsed = 1000, bool imu = true, int yaw = 1, int pitch = 1)
{
    const auto previous = test_timestamp_us;
    test_timestamp_us += elapsed;
    if (imu) MessageCenter::INS_State_Topic.Publish(ins);
    if (yaw >= 0) Receive(cfg.yaw, yaw, 0, 0);
    if (pitch >= 0) Receive(cfg.pitch, pitch, motor_position, motor_velocity);
    Gimbal_Update();
    if (test_timestamp_us / 10000 != previous / 10000) Class_DMMotor::ServiceAll();
}
static void Publish() { MessageCenter::Gimbal_Command_Topic.Publish(command); }
static void Init(bool tuned = true, bool integral = false)
{
    cfg = Gimbal_Default_Config();
    if (tuned) { cfg.yaw_speed_kp = 1; cfg.yaw_torque_limit = 2; }
    if (integral) { cfg.yaw_speed_ki = 1; cfg.yaw_integral_limit = 1; }
    CHECK(Gimbal_Init(cfg));
    CHECK(discrete.empty()); // No enable, zeroing or persistent writes during initialization.
    CHECK(receivers.size() == 2);
    CHECK(receivers[0].bus == &hfdcan1 && receivers[0].id == 0x101);
    CHECK(receivers[1].bus == &hfdcan1 && receivers[1].id == 0x102);
}
static void Activate()
{
    command.mode = GimbalMode::IMU; Publish();
    Tick(1000, true, 0, 0);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_ENABLING);
    CHECK(discrete.size() == 2 && discrete.back().data[7] == 0xfc);
    Tick();
    CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
}
static Struct_CAN_Tx_Msg Last(const Struct_Gimbal_Motor_Config &c)
{
    for (auto it = periodic.rbegin(); it != periodic.rend(); ++it)
        if (it->hfdcan == c.bus && it->id == c.id) return *it;
    CHECK(false);
    return {};
}
struct MIT { float p, v, kp, kd, torque; };
static MIT Decode(const Struct_Gimbal_Motor_Config &c)
{
    const auto f = Last(c);
    CHECK(f.len == 8);
    auto value = [](unsigned raw, float max, unsigned bits) {
        return raw * (2 * max) / ((1U << bits) - 1U) - max;
    };
    return {value((f.data[0] << 8) | f.data[1], c.position_max, 16),
        value((f.data[2] << 4) | (f.data[3] >> 4), c.velocity_max, 12),
        (((f.data[3] & 15) << 8) | f.data[4]) * 500.0f / 4095,
        ((f.data[5] << 4) | (f.data[6] >> 4)) * 5.0f / 4095,
        value(((f.data[6] & 15) << 8) | f.data[7], c.torque_max, 12)};
}
static void Zero()
{
    for (const auto &c : {cfg.yaw, cfg.pitch})
    {
        const auto f = Decode(c);
        CHECK(f.kp == 0 && f.kd == 0 && Near(f.torque, 0));
    }
}
static void TestConfig()
{
    auto base = Gimbal_Default_Config();
    for (int scenario = 0; scenario < 14; ++scenario)
    {
        auto c = base;
        switch (scenario)
        {
        case 0: c.yaw_gyro_sign = 0; break;
        case 1: c.pitch_speed_limit = 0; break;
        case 2: c.pitch_max = std::numeric_limits<float>::infinity(); break;
        case 3: c.pitch = c.yaw; break;
        case 4: c.yaw_angle_kp = -1; break;
        case 5: c.yaw_speed_ki = std::numeric_limits<float>::quiet_NaN(); break;
        case 6: c.yaw_torque_limit = -1; break;
        case 7: c.pitch_min = c.pitch_max; break;
        case 8: c.pitch_min = std::numeric_limits<float>::quiet_NaN(); break;
        case 9: c.pitch_kp = -1; break;
        case 10: c.pitch_speed_limit = 0; break;
        case 11: c.yaw_gyro_axis = static_cast<GimbalGyroAxis>(3); break;
        case 12: c.yaw_integral_limit = c.yaw_torque_limit + 1; break;
        case 13: c.pitch_motor_per_imu = 0; break;
        }
        CHECK(!Gimbal_Init(c));
        CHECK(Gimbal_GetStatus() == Gimbal_Status_CONFIG_ERROR);
        Gimbal_Update();
        CHECK(receivers.empty() && discrete.empty() && periodic.empty());
    }
}
static void TestControl()
{
    Init(); Activate();
    ins.yaw_rad = -179 * 3.14159265f / 180;
    command.yaw_angle_rad = 179 * 3.14159265f / 180;
    command.yaw_speed_rad_s = 0.5f;
    ins.gyro_z_rad_s = 0.1f;
    Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, (-2 * 3.14159265f / 180) * 8 + 0.5f - 0.1f));
    command.yaw_speed_rad_s = 100;
    Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, 2));
    command.yaw_speed_rad_s = -100;
    Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, -2));
}
static void TestPitch(bool reverse)
{
    cfg = Gimbal_Default_Config();
    cfg.pitch.reverse = reverse;
    cfg.yaw.reverse = reverse;
    cfg.yaw_speed_kp = 1;
    cfg.pitch_motor_per_imu = 2;
    CHECK(Gimbal_Init(cfg)); Activate();
    ins.pitch_rad = 0.1f;
    ins.gyro_y_rad_s = 0.2f;
    ins.gyro_x_rad_s = 10; // Must not accidentally use reference project's X-axis convention.
    motor_velocity = reverse ? -0.4f : 0.4f;
    command.yaw_angle_rad = 0.1f;
    command.pitch_angle_rad = 0.2f;
    command.pitch_speed_rad_s = 0.3f;
    Publish(); Tick();
    const float direction = reverse ? -1 : 1;
    const float logical_position = reverse ? -motor_position : motor_position;
    const float logical_velocity = reverse ? -motor_velocity : motor_velocity;
    auto output = Decode(cfg.pitch);
    CHECK(Near(Decode(cfg.yaw).torque, direction * 0.8f));
    CHECK(Near(output.p, direction * (logical_position + 0.2f)));
    CHECK(Near(output.v, direction * (logical_velocity + 0.2f), 0.03f));
    CHECK(Near(output.kp, 20, 0.13f) && Near(output.kd, 1, 0.002f));
    command.pitch_angle_rad = 100; command.pitch_speed_rad_s = 100;
    Publish(); Tick(); output = Decode(cfg.pitch);
    CHECK(Near(output.p, direction * cfg.pitch_max));
    CHECK(Near(output.v, direction * cfg.pitch_speed_limit, 0.03f));
    command.pitch_angle_rad = -100; command.pitch_speed_rad_s = -100;
    Publish(); Tick(); output = Decode(cfg.pitch);
    CHECK(Near(output.p, direction * cfg.pitch_min));
    CHECK(Near(output.v, -direction * cfg.pitch_speed_limit, 0.03f));
}
static void TestModes()
{
    Init(); Tick();
    CHECK(discrete.empty() && periodic.empty());
    discrete.clear(); Activate();
    ins.yaw_rad = 0.2f; ins.pitch_rad = 0.1f;
    command.mode = GimbalMode::LOCK; command.yaw_angle_rad = 2;
    Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, 0));
    CHECK(Near(Decode(cfg.pitch).p, motor_position));
    ins.yaw_rad = 0.25f; Tick();
    CHECK(Near(Decode(cfg.yaw).torque, -0.4f));
    command.yaw_angle_rad = 3; Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, -0.4f));
    command.mode = GimbalMode::DISABLED; Publish(); Tick(); Zero();
    discrete.clear();
    Tick(101000, true, -1, -1); DaemonManager::CheckAll();
    for (const auto &f : discrete) CHECK(f.data[7] == 0xfd);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_DISABLE);
}
static void TestRecovery()
{
    Init(true, true); Activate();
    command.yaw_angle_rad = 1; Publish();
    for (int i = 0; i < 200; ++i) Tick();
    command.yaw_angle_rad = 0; Publish(); Tick();
    CHECK(Decode(cfg.yaw).torque > 0.5f); // 外部输出证明积分已累积。
    command.yaw_angle_rad = 1; Publish(); Tick();
    Tick(11000, false);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_FAULT); Zero();
    ins.yaw_rad = 0.3f; ins.pitch_rad = 0.2f;
    Tick(999000, true, 0, 0);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_ENABLING);
    Tick(1000, true, 0, 0);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_ENABLING);
    Tick();
    for (int i = 0; i < 100; ++i) Tick();
    CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
    Tick(); CHECK(Near(Decode(cfg.yaw).torque, 0)); // Old yaw=1 must not return.
    command.yaw_angle_rad = 0.4f; command.pitch_angle_rad = 0.2f; Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, 0.8f));
    Tick(100000, true, -1, -1);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_ENABLING); Zero();
    Tick(1000000, true, 8, 1);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_FAULT);
    for (const auto &f : discrete) CHECK(f.data[7] != 0xfb && f.data[7] != 0xfe);
}
static void TestStableRequests()
{
    Init(); command.mode = GimbalMode::LOCK; Publish();
    Tick(); CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
    const auto commands = submit_attempts, targets = perform_attempts;
    for (int i = 0; i < 100; ++i) Tick();
    CHECK(submit_attempts == commands && perform_attempts == targets + 200);
    Tick(1000, true, 0, 1);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_ENABLING);
    Tick(); CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
    ins.pitch_rad = 0.3f;
    Tick(100000, true, -1, 1);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_ENABLING);
    CHECK(Near(Decode(cfg.yaw).torque, 0) && Decode(cfg.yaw).kp == 0 && Decode(cfg.yaw).kd == 0);
    Tick(1000, true, 0, 0);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_ENABLING);
    Tick(); CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
    CHECK(Near(Decode(cfg.pitch).p, motor_position));
    command.mode = GimbalMode::DISABLED; Publish(); Tick(); Zero();
    const auto stopped_commands = submit_attempts, stopped_targets = perform_attempts;
    for (int i = 0; i < 5; ++i) Tick(1000, true, 0, 0);
    CHECK(submit_attempts == stopped_commands && perform_attempts == stopped_targets);
}

static void TestOffline()
{
    Init(); command.mode = GimbalMode::IMU; Publish();
    Tick(1000, true, -1, -1);
    const auto attempts = submit_attempts;
    for (int i = 0; i < 19; ++i) Tick(1000, true, -1, -1);
    CHECK(submit_attempts == attempts);
    Tick(1980000, true, -1, -1);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_ENABLING); Zero();
    CHECK(submit_attempts == attempts);
    Tick(1000, true, 0, 0); Tick();
    CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
}
static void TestFailures()
{
    Init(); submit_ok = false;
    command.mode = GimbalMode::IMU; Publish(); Tick(1000, true, 0, 0);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_ENABLING && discrete.empty());
    const auto attempts = submit_attempts;
    submit_ok = true; Tick(1000, true, 0, 0);
    CHECK(submit_attempts == attempts);
    Tick(8000, true, 0, 0);
    CHECK(submit_attempts == attempts + 2);
    Tick(); CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
    perform_ok = false; Tick();
    CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
    const auto failed = perform_attempts;
    perform_ok = true; Tick(); CHECK(perform_attempts == failed + 2);
    command.mode = GimbalMode::DISABLED; Publish(); submit_ok = false; Tick(); Zero();
    CHECK(Gimbal_GetStatus() == Gimbal_Status_DISABLE);
    const auto before = submit_attempts;
    submit_ok = true; Tick(1000);
    CHECK(submit_attempts == before);
    Tick(20000);
    CHECK(submit_attempts == before + 2 && discrete.back().data[7] == 0xfd);
}
static void TestInputExpiry()
{
    Init(); Activate();
    Tick(10000, false);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
    Tick(1, false);
    CHECK(Gimbal_GetStatus() == Gimbal_Status_FAULT); Zero();
    Tick(); CHECK(Gimbal_GetStatus() == Gimbal_Status_READY);
}
static void TestDriver()
{
    cfg = Gimbal_Default_Config();
    Class_DMMotor yaw_motor;
    Class_DMMotor pitch_motor;
    CHECK(yaw_motor.Init(cfg.yaw.bus, cfg.yaw.id, cfg.yaw.feedback_id,
                         Enum_DMMotor_Mode::MIT));
    CHECK(pitch_motor.Init(cfg.pitch.bus, cfg.pitch.id, cfg.pitch.feedback_id,
                           Enum_DMMotor_Mode::MIT));
    Receive(cfg.yaw, 1, 1, 2);
    test_irq_mask = 1;
    const auto snapshot = yaw_motor.GetFeedbackSnapshot();
    CHECK(test_irq_mask == 1 && snapshot.online && snapshot.actual_enabled);
    CHECK(Near(snapshot.feedback.position, 1)); test_irq_mask = 0;
    test_timestamp_us += 100000;
    CHECK(!yaw_motor.GetFeedbackSnapshot().online); // No Daemon CheckAll required.
    perform_ok = false;
    CHECK(!yaw_motor.SetTorque(1));
    CHECK(!pitch_motor.SetMIT(0, 0, 20, 1, 0));
    perform_ok = true; CHECK(yaw_motor.SetTorque(100));
    CHECK(Near(Decode(cfg.yaw).torque, 0));
    CHECK(yaw_motor.RequestEnabled(true)); Receive(cfg.yaw, 1, 0, 0);
    CHECK(yaw_motor.SetTorque(100));
    CHECK(Near(Decode(cfg.yaw).torque, cfg.yaw.torque_max));
}
static void TestDmReconciliation()
{
    static Class_DMMotor motor;
    cfg = Gimbal_Default_Config();
    CHECK(motor.Init(cfg.yaw.bus, cfg.yaw.id, cfg.yaw.feedback_id,
                    Enum_DMMotor_Mode::MIT));
    CHECK(motor.RequestEnabled(true));
    const auto initial = submit_attempts;
    Class_DMMotor::ServiceAll(); // Never received feedback.
    CHECK(submit_attempts == initial);
    Receive(cfg.yaw, 0, 0, 0);
    Class_DMMotor::ServiceAll();
    CHECK(submit_attempts == initial + 1 && discrete.back().data[7] == 0xfc);
    Receive(cfg.yaw, 1, 0, 0);
    Class_DMMotor::ServiceAll(); CHECK(submit_attempts == initial + 1);
    CHECK(motor.RequestEnabled(false));
    const auto disabled = submit_attempts;
    Class_DMMotor::ServiceAll();
    CHECK(submit_attempts == disabled + 1 && discrete.back().data[7] == 0xfd);
    Receive(cfg.yaw, 0, 0, 0);
    Class_DMMotor::ServiceAll(); CHECK(submit_attempts == disabled + 1);
    CHECK(motor.RequestEnabled(true));
    Receive(cfg.yaw, 8, 0, 0);
    const auto faulted = submit_attempts;
    Class_DMMotor::ServiceAll(); CHECK(submit_attempts == faulted);
    CHECK(motor.RequestEnabled(false));
    const auto fault_disabled = submit_attempts;
    Class_DMMotor::ServiceAll(); CHECK(submit_attempts == fault_disabled);
    Receive(cfg.yaw, 1, 0, 0); test_timestamp_us += 100000;
    Class_DMMotor::ServiceAll(); CHECK(submit_attempts == fault_disabled);
    CHECK(motor.RequestEnabled(true));
    const auto offline = submit_attempts;
    for (int i = 0; i < 20; ++i) { test_timestamp_us += 10000; Class_DMMotor::ServiceAll(); }
    CHECK(submit_attempts == offline);
    for (const auto &frame : discrete) CHECK(frame.data[7] != 0xfb);
}
static void TestDmGimbalException()
{
    Init();
    Receive(cfg.yaw, 1, 0, 0);
    Receive(cfg.pitch, 1, 0, 0);
    discrete.clear();
    test_timestamp_us += 101000;
    DaemonManager::CheckAll();
    Class_DMMotor::ServiceAll();
    CHECK(discrete.empty());
}
static void TestFeedbackPeriod()
{
    Init();
    const auto start = MessageCenter::Gimbal_Feedback_Topic.Sequence();
    for (int i = 0; i < 9; ++i) Tick();
    CHECK(MessageCenter::Gimbal_Feedback_Topic.Sequence() == start);
    Tick();
    CHECK(MessageCenter::Gimbal_Feedback_Topic.Sequence() == start + 1);
    for (int i = 0; i < 10; ++i) Tick();
    CHECK(MessageCenter::Gimbal_Feedback_Topic.Sequence() == start + 2);
}
static void TestDmEdges()
{
    static Class_DMMotor motor;
    cfg = Gimbal_Default_Config();
    CHECK(motor.Init(cfg.yaw.bus, cfg.yaw.id, cfg.yaw.feedback_id,
                    Enum_DMMotor_Mode::MIT, false, cfg.yaw.position_max,
                    cfg.yaw.velocity_max, cfg.yaw.torque_max));
    CHECK(motor.RequestEnabled(false));
    CHECK(!motor.GetFeedbackSnapshot().requested_enabled);
    Receive(cfg.yaw, 1, 0, 0); Class_DMMotor::ServiceAll();
    CHECK(submit_attempts == 0 && perform_attempts == 0); // No lifecycle activation.
    CHECK(motor.RequestEnabled(true));
    CHECK(motor.GetFeedbackSnapshot().requested_enabled);
    CHECK(submit_attempts == 1 && perform_attempts == 0 && discrete.back().data[7] == 0xfc);
    CHECK(motor.SetTorque(2));
    const auto normal = Last(cfg.yaw);
    for (int i = 0; i < 1000; ++i) CHECK(motor.RequestEnabled(true));
    CHECK(submit_attempts == 1 && perform_attempts == 1);
    CHECK(std::memcmp(Last(cfg.yaw).data, normal.data, 8) == 0);
    CHECK(Near(Decode(cfg.yaw).torque, 2));
    CHECK(motor.RequestEnabled(false));
    CHECK(!motor.GetFeedbackSnapshot().ready && !motor.GetFeedbackSnapshot().requested_enabled);
    CHECK(submit_attempts == 2 && perform_attempts == 2 && discrete.back().data[7] == 0xfd);
    CHECK(operations[operations.size() - 2] == 'P' && operations.back() == 'D');
    CHECK(Near(Decode(cfg.yaw).torque, 0) && Decode(cfg.yaw).kp == 0 && Decode(cfg.yaw).kd == 0);
    for (int i = 0; i < 1000; ++i) CHECK(motor.RequestEnabled(false));
    CHECK(submit_attempts == 2 && perform_attempts == 2);
}
static void TestDmRequestFailures()
{
    static Class_DMMotor motor;
    cfg = Gimbal_Default_Config();
    CHECK(motor.Init(cfg.yaw.bus, cfg.yaw.id, cfg.yaw.feedback_id, Enum_DMMotor_Mode::MIT));
    for (int command_ok = 0; command_ok < 2; ++command_ok)
    {
        for (int safe_ok = 0; safe_ok < 2; ++safe_ok)
        {
            submit_ok = command_ok; perform_ok = safe_ok;
            const auto commands = submit_attempts, targets = perform_attempts;
            CHECK(motor.RequestEnabled(true) == submit_ok);
            CHECK(motor.GetFeedbackSnapshot().requested_enabled);
            CHECK(submit_attempts == commands + 1 && perform_attempts == targets);
            CHECK(motor.RequestEnabled(true)); // Even after a failed edge submission.
            CHECK(submit_attempts == commands + 1 && perform_attempts == targets);
            CHECK(motor.RequestEnabled(false) == (submit_ok && perform_ok));
            CHECK(!motor.GetFeedbackSnapshot().requested_enabled);
            CHECK(submit_attempts == commands + 2 && perform_attempts == targets + 1);
            CHECK(operations[operations.size() - 2] == 'P' && operations.back() == 'D');
            CHECK(motor.RequestEnabled(false));
            CHECK(submit_attempts == commands + 2 && perform_attempts == targets + 1);
        }
    }
    submit_ok = false; perform_ok = true;
    CHECK(!motor.RequestEnabled(true));
    CHECK(motor.RequestEnabled(true));
    Receive(cfg.yaw, 0, 0, 0);
    submit_ok = true;
    const auto before = submit_attempts;
    Class_DMMotor::ServiceAll();
    CHECK(submit_attempts == before + 1 && discrete.back().data[7] == 0xfc);
}
static void TestDmSafeSetters()
{
    static Class_DMMotor motors[4];
    registration_limit = 4;
    cfg = Gimbal_Default_Config();
    for (int index = 0; index < 4; ++index)
    {
        auto config = cfg.yaw;
        config.id = index + 1; config.feedback_id = 0x101 + index;
        auto &motor = motors[index];
        CHECK(motor.Init(config.bus, config.id, config.feedback_id,
                        static_cast<Enum_DMMotor_Mode>(index + 1), true,
                        config.position_max, config.velocity_max, config.torque_max));
        auto publish = [&]() {
            switch (index)
            {
            case 0: CHECK(motor.SetMIT(1, 2, 20, 1, 2)); break;
            case 1: CHECK(motor.SetPositionSpeed(1, 2)); break;
            case 2: CHECK(motor.SetSpeed(2)); break;
            case 3: CHECK(motor.SetForcePosition(1, 2, 0.5f)); break;
            }
            return periodic.back();
        };
        auto check_safe = [&](const Struct_CAN_Tx_Msg &frame) {
            CHECK(frame.id == static_cast<uint32_t>(index * 0x100 + config.id));
            if (index == 0)
            {
                CHECK(frame.data[3] % 16 == 0 && frame.data[4] == 0 && frame.data[5] == 0);
                CHECK((frame.data[6] >> 4) == 0);
            }
            else
            {
                float value; std::memcpy(&value, frame.data, sizeof(value));
                CHECK(Near(value, index == 2 ? 0.0f : -motor.feedback.position));
                if (index == 1)
                {
                    float velocity; std::memcpy(&velocity, frame.data + 4, sizeof(velocity));
                    CHECK(velocity == 0.0f); // Reverse can encode negative zero.
                }
                else for (unsigned byte = 4; byte < frame.len; ++byte) CHECK(frame.data[byte] == 0);
            }
        };
        for (int state : {1, 0, 8})
        {
            Receive(config, state, 0.3f, 0.2f);
            check_safe(publish()); // requested=false always gates output.
        }
        CHECK(motor.RequestEnabled(true));
        for (int state : {0, 8})
        {
            Receive(config, state, 0.3f, 0.2f); check_safe(publish());
        }
        Receive(config, 1, 0.3f, 0.2f);
        CHECK(motor.GetFeedbackSnapshot().ready);
        const auto normal = publish();
        test_timestamp_us += 100000;
        const auto safe = publish(); check_safe(safe);
        CHECK(std::memcmp(normal.data, safe.data, safe.len) != 0);
        if (index == 0)
        {
            CHECK(motor.SetTorque(2));
            CHECK(std::memcmp(periodic.back().data, safe.data, safe.len) == 0);
        }
    }
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!std::strcmp(argv[1], "config")) TestConfig();
    else if (!std::strcmp(argv[1], "init_failure"))
    {
        cfg = Gimbal_Default_Config(); registration_ok = false;
        CHECK(!Gimbal_Init(cfg)); command.mode = GimbalMode::IMU; Publish(); Tick();
        CHECK(Gimbal_GetStatus() == Gimbal_Status_CONFIG_ERROR && discrete.empty());
    }
    else if (!std::strcmp(argv[1], "partial_init"))
    {
        cfg = Gimbal_Default_Config(); registration_limit = 1;
        CHECK(!Gimbal_Init(cfg)); Tick();
        CHECK(Gimbal_GetStatus() == Gimbal_Status_CONFIG_ERROR);
        CHECK(discrete.empty() && periodic.empty());
    }
    else if (!std::strcmp(argv[1], "stable_requests")) TestStableRequests();
    else if (!std::strcmp(argv[1], "control")) TestControl();
    else if (!std::strcmp(argv[1], "pitch")) TestPitch(false);
    else if (!std::strcmp(argv[1], "direction")) TestPitch(true);
    else if (!std::strcmp(argv[1], "modes")) TestModes();
    else if (!std::strcmp(argv[1], "recovery")) TestRecovery();
    else if (!std::strcmp(argv[1], "offline")) TestOffline();
    else if (!std::strcmp(argv[1], "failures")) TestFailures();
    else if (!std::strcmp(argv[1], "input_expiry")) TestInputExpiry();
    else if (!std::strcmp(argv[1], "driver")) TestDriver();
    else if (!std::strcmp(argv[1], "dm_reconciliation")) TestDmReconciliation();
    else if (!std::strcmp(argv[1], "dm_gimbal_exception")) TestDmGimbalException();
    else if (!std::strcmp(argv[1], "feedback_period")) TestFeedbackPeriod();
    else if (!std::strcmp(argv[1], "dm_edges")) TestDmEdges();
    else if (!std::strcmp(argv[1], "dm_request_failures")) TestDmRequestFailures();
    else if (!std::strcmp(argv[1], "dm_safe_setters")) TestDmSafeSetters();
    else if (!std::strcmp(argv[1], "defaults"))
    {
        Init(false); Activate(); command.yaw_angle_rad = 2; Publish(); Tick();
        CHECK(Near(Decode(cfg.yaw).torque, 0));
    }
    else CHECK(false);
    CHECK(test_irq_mask == 0);
    std::printf("PASS gimbal %s\n", argv[1]);
}
