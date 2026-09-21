#include "SpeedPlanning.h"

#include <math.h>

static float SpeedPlanning_LimitNonNegative(float value)
{
    return value > 0.0F ? value : 0.0F;
}

void SpeedPlanning_Init(SpeedPlanningState* state, float initial_speed)
{
    if (state == 0)
    {
        return;
    }

    state->current_speed = initial_speed;
    state->target_speed_last = initial_speed;
    state->time_since_change = 0.0F;
    state->transition_active = 0U;
    state->transition_accelerating = 0U;
}

float SpeedPlanning_SCurve(float normalized_time)
{
    float time_squared;
    float time_cubed;
    float time_fourth;
    float time_fifth;

    if (normalized_time <= 0.0F)
    {
        return 0.0F;
    }
    if (normalized_time >= 1.0F)
    {
        return 1.0F;
    }

    time_squared = normalized_time * normalized_time;
    time_cubed = time_squared * normalized_time;
    time_fourth = time_squared * time_squared;
    time_fifth = time_cubed * time_squared;

    return 10.0F * time_cubed - 15.0F * time_fourth + 6.0F * time_fifth;
}

float SpeedPlanning_ApplyDeadbandExpo(float input, float input_max, float deadband_ratio, float expo)
{
    float normalized_input;
    float magnitude;
    float shaped_magnitude;

    if (input_max <= 0.0F)
    {
        return 0.0F;
    }

    normalized_input = input / input_max;
    if (normalized_input > 1.0F)
    {
        normalized_input = 1.0F;
    }
    else if (normalized_input < -1.0F)
    {
        normalized_input = -1.0F;
    }

    deadband_ratio = fminf(fmaxf(deadband_ratio, 0.0F), 0.99F);
    expo = fminf(fmaxf(expo, 0.0F), 1.0F);
    magnitude = fabsf(normalized_input);
    if (magnitude <= deadband_ratio)
    {
        return 0.0F;
    }

    magnitude = (magnitude - deadband_ratio) / (1.0F - deadband_ratio);
    shaped_magnitude = (1.0F - expo) * magnitude + expo * magnitude * magnitude * magnitude;
    return normalized_input < 0.0F ? -shaped_magnitude : shaped_magnitude;
}

float SpeedPlanning_Update(float target_speed, SpeedPlanningState* state, float dt, float transition_time, float target_threshold)
{
    float target_delta;
    float normalized_time;
    float interpolation;

    if (state == 0)
    {
        return target_speed;
    }

    dt = SpeedPlanning_LimitNonNegative(dt);
    transition_time = SpeedPlanning_LimitNonNegative(transition_time);
    target_threshold = SpeedPlanning_LimitNonNegative(target_threshold);
    target_delta = target_speed - state->target_speed_last;

    if (transition_time <= 0.0F)
    {
        state->current_speed = target_speed;
        state->transition_active = 0U;
    }
    else
    {
        if (state->transition_active == 0U && fabsf(target_delta) > target_threshold)
        {
            state->time_since_change = 0.0F;
            state->transition_active = 1U;
        }

        normalized_time = state->time_since_change / transition_time;
        interpolation = SpeedPlanning_SCurve(normalized_time);
        state->current_speed += (target_speed - state->current_speed) * interpolation;

        if (state->time_since_change + dt >= transition_time)
        {
            state->current_speed = target_speed;
            state->transition_active = 0U;
        }
    }

    state->target_speed_last = target_speed;
    state->time_since_change += dt;

    return state->current_speed;
}

float SpeedPlanning_UpdateAsymmetric(float target_speed, SpeedPlanningState* state, float dt, float acceleration_time, float deceleration_time, float target_threshold)
{
    float planned_target;
    float target_delta;
    float transition_time;

    if (state == 0)
    {
        return target_speed;
    }

    planned_target = target_speed;

    /* Reverse in two stages: brake to zero before accelerating the other way. */
    if (state->current_speed * target_speed < 0.0F)
    {
        planned_target = 0.0F;
    }

    target_threshold = SpeedPlanning_LimitNonNegative(target_threshold);
    target_delta = planned_target - state->target_speed_last;

    if (state->transition_active == 0U || fabsf(target_delta) > target_threshold)
    {
        uint8_t accelerating = fabsf(planned_target) > fabsf(state->current_speed) ? 1U : 0U;

        if (state->transition_active != 0U && accelerating != state->transition_accelerating)
        {
            state->time_since_change = 0.0F;
        }
        state->transition_accelerating = accelerating;
    }

    if (state->transition_accelerating != 0U)
    {
        transition_time = acceleration_time;
    }
    else
    {
        transition_time = deceleration_time;
    }

    return SpeedPlanning_Update(planned_target, state, dt, transition_time, target_threshold);
}

float SpeedPlanning_UpdateRateLimited(float target_speed, SpeedPlanningState* state, float dt,
                                      float acceleration_limit, float deceleration_limit,
                                      float release_deceleration_limit, float reversal_deceleration_limit,
                                      float zero_threshold)
{
    float planned_target;
    float rate_limit;
    float speed_delta;
    float maximum_delta;

    if (state == 0)
    {
        return target_speed;
    }

    dt = SpeedPlanning_LimitNonNegative(dt);
    zero_threshold = SpeedPlanning_LimitNonNegative(zero_threshold);
    planned_target = fabsf(target_speed) <= zero_threshold ? 0.0F : target_speed;

    if (state->current_speed * planned_target < 0.0F)
    {
        planned_target = 0.0F;
        rate_limit = reversal_deceleration_limit;
    }
    else if (planned_target == 0.0F)
    {
        rate_limit = release_deceleration_limit;
    }
    else if (fabsf(planned_target) > fabsf(state->current_speed))
    {
        rate_limit = acceleration_limit;
    }
    else
    {
        rate_limit = deceleration_limit;
    }

    rate_limit = SpeedPlanning_LimitNonNegative(rate_limit);
    speed_delta = planned_target - state->current_speed;
    maximum_delta = rate_limit * dt;

    if (rate_limit <= 0.0F || fabsf(speed_delta) <= maximum_delta)
    {
        state->current_speed = planned_target;
    }
    else if (speed_delta > 0.0F)
    {
        state->current_speed += maximum_delta;
    }
    else
    {
        state->current_speed -= maximum_delta;
    }

    state->target_speed_last = target_speed;
    state->time_since_change += dt;
    state->transition_active = state->current_speed != target_speed ? 1U : 0U;
    state->transition_accelerating = rate_limit == acceleration_limit ? 1U : 0U;
    return state->current_speed;
}
