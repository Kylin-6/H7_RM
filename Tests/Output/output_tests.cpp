#include "RobotCmd.h"
#include "message_center.h"
#include <cstdio>
#include <cstdlib>

#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); std::exit(1); } } while (0)
uint32_t test_irq_mask;
uint64_t test_timestamp_us;
extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void) { return test_timestamp_us; }

int main()
{
    Output<GimbalCmd> unbound;
    CHECK(!unbound.IsBound());
    Topic<GimbalCmd> gimbal_topic;
    Topic<ChassisCmd> chassis_topic;
    Topic<ShootCmd> shoot_topic;
    LocalPublisher<GimbalCmd> gimbal(gimbal_topic);
    LocalPublisher<ChassisCmd> chassis(chassis_topic);
    LocalPublisher<ShootCmd> shoot(shoot_topic);
    CHECK(gimbal.Bind().IsBound() && chassis.Bind().IsBound() && shoot.Bind().IsBound());
    CHECK(!RobotCmd_Init(unbound, chassis.Bind(), shoot.Bind()));
    CHECK(!RobotCmd_Init(gimbal.Bind(), Output<ChassisCmd>{}, shoot.Bind()));
    CHECK(!RobotCmd_Init(gimbal.Bind(), chassis.Bind(), Output<ShootCmd>{}));
    CHECK(!RobotCmd_Init(Output<GimbalCmd>{nullptr,
        [](void *, const GimbalCmd &) {}}, chassis.Bind(), shoot.Bind()));
    CHECK(RobotCmd_Init(gimbal.Bind(), chassis.Bind(), shoot.Bind()));
    RobotCmd_Update();
    CHECK(gimbal_topic.Sequence() == 1 && chassis_topic.Sequence() == 1 &&
          shoot_topic.Sequence() == 1);
    ChassisFeedback feedback{};
    feedback.enabled = true;
    MessageCenter::Chassis_Feedback_Topic.PublishAt(feedback, 1000);
    test_timestamp_us = 101000;
    ChassisFeedback result{};
    CHECK(RobotCmd_GetChassisFeedback(result) && result.enabled);
    test_timestamp_us = 101001;
    result.enabled = false;
    CHECK(!RobotCmd_GetChassisFeedback(result) && !result.enabled);
    std::puts("PASS output binding and feedback age");
}
