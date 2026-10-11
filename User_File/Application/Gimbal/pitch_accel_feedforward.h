#ifndef PITCH_ACCEL_FEEDFORWARD_H
#define PITCH_ACCEL_FEEDFORWARD_H

#include "Gimbal_Config.h"
#include "dvc_dm_imu.h"
#include <cmath>

/** 云台比力扣除重力后的惯性力矩前馈；不重新解算姿态，不修改 INS。 */
class Class_PitchAccelFeedforward
{
  public:
    float linear_m_s2[3]{};
    float filtered_m_s2[3]{};
    float torque_nm = 0.0f;
    bool fresh = false;
    bool enabled = false;

    void Reset()
    {
        has_sample = false;
        fresh = enabled = false;
        torque_nm = 0.0f;
        for (unsigned i = 0; i < 3; ++i)
            linear_m_s2[i] = filtered_m_s2[i] = 0.0f;
    }

    float Update(const Struct_DM_IMU_Accel_Snapshot &sample, float pitch, float roll,
                 uint64_t now_us, float control_dt_s, const Struct_Pitch_AccelFeedforward_Config &c)
    {
        if (!sample.valid || now_us < sample.timestamp_us || now_us - sample.timestamp_us > c.max_age_us ||
            !std::isfinite(pitch) || !std::isfinite(roll))
        {
            Reset();
            return 0.0f;
        }
        for (float value : sample.accel_m_s2)
        {
            if (!std::isfinite(value))
            {
                Reset();
                return 0.0f;
            }
        }
        fresh = true;
        enabled = c.mass_height_kg_m != 0.0f || c.mass_forward_kg_m != 0.0f;
        if (!has_sample || sample.sequence != last_sequence)
        {
            const bool restart = !has_sample || sample.timestamp_us <= last_us ||
                                 sample.timestamp_us - last_us > c.max_age_us;
            const float dt_s = restart ? 0.0f : static_cast<float>(sample.timestamp_us - last_us) * 1e-6f;
            // ZYX 欧拉角、右手系，静止比力为 R^T*[0,0,g]。
            // 启用系数前须用静止多角度验证重力残差接近零。
            const float gravity[3] = {-9.80665f * std::sin(pitch),
                                     9.80665f * std::sin(roll) * std::cos(pitch),
                                     9.80665f * std::cos(roll) * std::cos(pitch)};
            for (unsigned i = 0; i < 3; ++i)
            {
                linear_m_s2[i] = sample.accel_m_s2[i] - gravity[i];
                const float bounded = Limit(linear_m_s2[i], c.accel_limit_m_s2);
                filtered_m_s2[i] = restart ? bounded : filtered_m_s2[i] +
                    dt_s / (c.filter_tau_s + dt_s) * (bounded - filtered_m_s2[i]);
            }
            last_us = sample.timestamp_us;
            last_sequence = sample.sequence;
            has_sample = true;
            if (restart)
            {
                torque_nm = 0.0f;
                return 0.0f;
            }
        }
        // 补偿惯性扰动力矩：+m*r_z*a_x - m*r_x*a_z，IMU +Y 正方向。
        const float target = enabled ? Limit(c.mass_height_kg_m * filtered_m_s2[0] -
                                             c.mass_forward_kg_m * filtered_m_s2[2], c.torque_limit_nm) : 0.0f;
        torque_nm += Limit(target - torque_nm, c.torque_slew_nm_s * control_dt_s);
        return torque_nm;
    }

  private:
    static float Limit(float value, float limit)
    {
        return value < -limit ? -limit : (value > limit ? limit : value);
    }
    bool has_sample = false;
    uint32_t last_sequence = 0U;
    uint64_t last_us = 0U;
};
#endif
