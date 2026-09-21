#pragma once
#include <stdint.h>
extern uint64_t test_sample_us;
class Class_BMI088
{
public:
    uint64_t Get_Last_Sample_Timestamp_Us() const
    {
        return test_sample_us;
    }
};
extern Class_BMI088 BSP_BMI088;
