#include "power_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int checks;
static int failures;

static void Check(bool condition, const char *label)
{
    checks++;
    if (!condition)
    {
        failures++;
        printf("FAIL: %s\n", label);
    }
}

static void Near(float actual, double expected, double tolerance, const char *label)
{
    Check(isfinite(actual) && fabs(actual - expected) <= tolerance, label);
}

static PowerControlConfig_t Config(bool identify)
{
    PowerControlConfig_t config = {};
    config.motor_count = 1;
    config.static_power_w = 2.0f;
    config.friction_coefficient = 0.1f;
    config.copper_coefficient = 0.4f;
    config.reserve_power_w = 0.0f;
    config.enable_rls = identify;
    config.rls_forgetting_factor = 1.0f;
    config.rls_initial_covariance = 100.0f;
    config.friction_coefficient_max = 2.0f;
    config.copper_coefficient_max = 2.0f;
    return config;
}

static void Limit()
{
    PowerControl control;
    PowerControlConfig_t config = Config(false);
    Check(control.Init(&config), "init");
    PowerControlInput_t input = {};
    input.desired_torque_nm[0] = 1.0f;
    input.speed_rad_s[0] = 10.0f;
    input.power_limit_w = 20.0f;
    Check(control.Set_Input(&input), "input");
    Check(control.TIM_Calculate_PeriodElapsedCallback(), "calculate unrestricted");
    PowerControlOutput_t out = control.Get_Output();
    Near(out.scale, 1.0, 0.0, "unrestricted scale");
    Near(out.estimated_power_w, 13.4, 0.00001, "unrestricted power");
    Check(!out.limited, "unrestricted flag");
    Check(!control.TIM_Calculate_PeriodElapsedCallback(), "stale input not reused");
    Near(control.Get_Output().torque_nm[0], 0.0, 0.0, "stale input clears torque");

    input.power_limit_w = 8.0f;
    Check(control.Set_Input(&input), "input limited");
    Check(control.TIM_Calculate_PeriodElapsedCallback(), "calculate limited");
    out = control.Get_Output();
    Check(out.limited && out.scale > 0.0f && out.scale < 1.0f, "limited scale range");
    Near(out.torque_nm[0], out.scale, 0.000001, "output scaled torque");
    Near(out.estimated_power_w, 8.0, 0.00002, "quadratic budget solution");

    input.power_limit_w = 2.5f;
    Check(control.Set_Input(&input), "input below baseline");
    Check(control.TIM_Calculate_PeriodElapsedCallback(), "calculate below baseline");
    out = control.Get_Output();
    Near(out.scale, 0.0, 0.0, "zero commanded torque");
    Near(out.torque_nm[0], 0.0, 0.0, "zero output torque");
    Near(out.estimated_power_w, 3.0, 0.00001, "baseline cannot be removed by torque scaling");
    Check(out.budget_below_baseline, "unachievable budget indicated");

    input.desired_torque_nm[0] = -2.0f;
    input.power_limit_w = 20.0f;
    Check(control.Set_Input(&input), "braking input");
    Check(control.TIM_Calculate_PeriodElapsedCallback(), "braking calculation");
    out = control.Get_Output();
    Near(out.estimated_power_w, 4.6, 0.00001, "braking gives no assumed regenerative credit");

    config = Config(false);
    config.motor_count = 4;
    config.static_power_w = 0.0f;
    config.friction_coefficient = 0.0f;
    config.copper_coefficient = 1.0f;
    config.reserve_power_w = 5.0f;
    Check(control.Init(&config), "four motor init");
    input = {};
    for (int i = 0; i < 4; i++)
    {
        input.desired_torque_nm[i] = 1.0f;
    }
    input.power_limit_w = 7.0f;
    Check(control.Set_Input(&input), "four motor input");
    Check(control.TIM_Calculate_PeriodElapsedCallback(), "four motor calculation");
    out = control.Get_Output();
    Near(out.scale, sqrt(0.5), 0.000001, "four motor quadratic scale");
    Near(out.estimated_power_w, 2.0, 0.000001, "reserve held from power budget");
    for (int i = 0; i < 4; i++)
    {
        Near(out.torque_nm[i], sqrt(0.5), 0.000001, "four outputs limited equally");
    }
}

static void Identification()
{
    PowerControl control;
    PowerControlConfig_t config = Config(true);
    Check(control.Init(&config), "RLS init");
    for (int i = 0; i < 300; i++)
    {
        const float torque[1] = {0.3f + (i % 7) * 0.19f};
        const float speed[1] = {3.0f + (i % 11) * 1.7f};
        const float measured = 2.0f + torque[0] * speed[0] +
                               0.3f * speed[0] + 0.8f * torque[0] * torque[0];
        Check(control.Update_Identification(measured, torque, speed), "valid RLS sample");
    }
    float friction = 0.0f;
    float copper = 0.0f;
    Check(control.Get_Model_Coefficients(&friction, &copper), "get RLS model");
    Near(friction, 0.3, 0.001, "friction converges");
    Near(copper, 0.8, 0.001, "copper converges");

    const float torque[1] = {1.0f};
    const float speed[1] = {5.0f};
    Check(!control.Update_Identification(NAN, torque, speed), "nonfinite measurement rejected");
    Check(!control.Update_Identification(0.0f, torque, speed), "negative residual rejected");
    const float braking[1] = {-1.0f};
    Check(!control.Update_Identification(10.0f, braking, speed), "braking sample rejected");
    Check(!control.Update_Identification(10000.0f, torque, speed), "out-of-range model rejected");
    float after_friction = 0.0f;
    float after_copper = 0.0f;
    Check(control.Get_Model_Coefficients(&after_friction, &after_copper), "get unchanged model");
    Near(after_friction, friction, 0.0, "bad sample preserves friction");
    Near(after_copper, copper, 0.0, "bad sample preserves copper");

    control.Reset_Identification();
    Check(control.Get_Model_Coefficients(&friction, &copper), "get reset model");
    Near(friction, config.friction_coefficient, 0.0, "friction reset");
    Near(copper, config.copper_coefficient, 0.0, "copper reset");
}

static void Invalid()
{
    PowerControl control;
    PowerControlConfig_t config = Config(false);
    config.motor_count = 0;
    Check(!control.Init(&config), "zero motor count rejected");
    config = Config(false);
    Check(control.Init(&config), "valid init after failure");
    PowerControlInput_t input = {};
    input.desired_torque_nm[0] = 1.0f;
    input.speed_rad_s[0] = 2.0f;
    input.power_limit_w = 10.0f;
    Check(control.Set_Input(&input), "valid input");
    Check(control.TIM_Calculate_PeriodElapsedCallback(), "valid calculate");
    const PowerControlOutput_t before = control.Get_Output();
    input.desired_torque_nm[0] = NAN;
    Check(!control.Set_Input(&input), "nonfinite torque rejected");
    Check(!control.TIM_Calculate_PeriodElapsedCallback(), "invalid input cannot calculate");
    const PowerControlOutput_t after = control.Get_Output();
    Check(before.torque_nm[0] != 0.0f, "previous command was nonzero");
    Near(after.torque_nm[0], 0.0, 0.0, "invalid input clears output");
    config = Config(true);
    config.rls_forgetting_factor = 0.0f;
    Check(!control.Init(&config), "invalid RLS config rejected");
    input.desired_torque_nm[0] = 1.0f;
    Check(control.Set_Input(&input), "old valid config survives failed reinit");
    Check(control.TIM_Calculate_PeriodElapsedCallback(), "old valid config still calculates");
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        return 2;
    }
    if (strcmp(argv[1], "limit") == 0)
    {
        Limit();
    }
    else if (strcmp(argv[1], "identification") == 0)
    {
        Identification();
    }
    else if (strcmp(argv[1], "invalid") == 0)
    {
        Invalid();
    }
    else
    {
        return 2;
    }
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
