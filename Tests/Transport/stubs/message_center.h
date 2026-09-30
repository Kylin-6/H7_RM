#pragma once
#include "message_types.h"
#include "topic.h"
#include <stdint.h>
template<typename T> struct TestTopic : Topic<T> {
    T value{};
    uint64_t at = 0;
    unsigned count = 0;
    void PublishAt(const T &v, uint64_t t) { Topic<T>::PublishAt(v, t); value = v; at = t; ++count; }
};
namespace MessageCenter {
extern TestTopic<ChassisCmd> Chassis_Command_Topic;
extern TestTopic<ChassisFeedback> Chassis_Feedback_Topic;
}
