#pragma once
#include "message_types.h"
#include <stdint.h>
template<typename T> struct TestTopic {
    T value{};
    uint64_t at = 0;
    unsigned count = 0;
    void PublishAt(const T &v, uint64_t t) { value = v; at = t; ++count; }
};
template<typename T> struct Subscriber {
    explicit Subscriber(TestTopic<T> &topic) : topic(topic) {}
    bool Read(T &out) { if (topic.count == read_count) return false; out = topic.value; read_count = topic.count; return true; }
    TestTopic<T> &topic;
    unsigned read_count = 0;
};
namespace MessageCenter {
extern TestTopic<ChassisCmd> Chassis_Command_Topic;
extern TestTopic<ChassisFeedback> Chassis_Feedback_Topic;
}
