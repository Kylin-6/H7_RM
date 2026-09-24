#include "spin_mode.h"
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

static SpinConfig_t Config(SpinMode_e mode)
{
    const SpinConfig_t config = {0.0f, mode};
    return config;
}

class SpinProbe : public SpinMode
{
  public:
    uint32_t Get_Count() const
    {
        return SpinMode_FSM.Status[SpinMode_FSM.Get_Now_Status_Serial()].Count_Time;
    }
};

static void Transform()
{
    SpinMode spin;
    Check(spin.Init(), "default init");
    const float angles[] = {0.0f, 0.5f * PI, -0.5f * PI, PI};
    const float expected_x[] = {1.0f, -2.0f, 2.0f, -1.0f};
    const float expected_y[] = {2.0f, 1.0f, -1.0f, -2.0f};
    spin.Set_MoveTarget(1.0f, 2.0f, -3.0f);
    for (int i = 0; i < 4; i++)
    {
        spin.Set_theta(angles[i]);
        spin.TIM_Calculate_PeriodElapsedCallback();
        SpinOutput_t out = spin.Get_Output();
        Near(out.x, expected_x[i], 0.00001, "cardinal x");
        Near(out.y, expected_y[i], 0.00001, "cardinal y");
    }

    SpinConfig_t config = Config(SpinMode_GIMBAL_LOCK);
    config.zero_point = 0.375f;
    Check(spin.Init(&config), "offset init");
    for (int i = -80; i <= 80; i++)
    {
        const float angle = i * 0.217f;
        const float vx = (i % 7) * 0.31f;
        const float vy = (i % 11) * -0.17f;
        const float measured = angle + config.zero_point;
        spin.Set_theta(measured);
        spin.Set_MoveTarget(vx, vy, 0.7f);
        spin.TIM_Calculate_PeriodElapsedCallback();
        SpinOutput_t out = spin.Get_Output();
        const double reference_angle = (double)measured - config.zero_point;
        Near(out.x, vx * cos(reference_angle) - vy * sin(reference_angle), 0.00002, "reference x");
        Near(out.y, vx * sin(reference_angle) + vy * cos(reference_angle), 0.00002, "reference y");
        Near(out.x * out.x + out.y * out.y, vx * vx + vy * vy, 0.00002, "rotation preserves norm");
        spin.Set_theta(measured + 4.0f * PI);
        spin.TIM_Calculate_PeriodElapsedCallback();
        SpinOutput_t periodic = spin.Get_Output();
        Near(periodic.x, out.x, 0.00002, "whole turn x");
        Near(periodic.y, out.y, 0.00002, "whole turn y");
    }
    spin.Set_theta(config.zero_point);
    spin.Set_MoveTarget(1.0f, 0.0f, 0.0f);
    spin.TIM_Calculate_PeriodElapsedCallback();
    Near(spin.Get_Output().x, 1.0, 0.00001, "calibrated forward x");
    Near(spin.Get_Output().y, 0.0, 0.00001, "calibrated forward y");
}

static void Angular()
{
    SpinMode spin;
    SpinConfig_t config = Config(SpinMode_CHASSIS_FOLLOW);
    Check(spin.Init(&config), "angular init");
    const float angles[] = {0.0f, 0.375f, -0.375f, PI - 0.01f, -PI + 0.01f, 4.0f * PI};
    const float speeds[] = {0.0f, 0.5f, -0.5f, 99.0f, -99.0f};
    for (int mode = 0; mode < 3; mode++)
    {
        spin.Set_Spin_Mode((SpinMode_e)mode);
        for (int speed = 0; speed < 5; speed++)
        {
            spin.Set_MoveTarget(0.0f, 0.0f, speeds[speed]);
            for (int angle = 0; angle < 6; angle++)
            {
                spin.Set_theta(angles[angle]);
                spin.TIM_Calculate_PeriodElapsedCallback();
                SpinOutput_t out = spin.Get_Output();
                Near(out.w, speeds[speed], 0.0, "caller owns angular speed at any angle");
                Near(out.forward, mode == SpinMode_GIMBAL_FOLLOW ? 0.0 : -speeds[speed],
                     0.0, "compensation uses caller angular speed");
                Near(out.w + out.forward, mode == SpinMode_GIMBAL_FOLLOW ? speeds[speed] : 0.0,
                     0.0, "feedforward preserves the requested gimbal world rate");
            }
        }
    }
}

static void Modes()
{
    SpinMode spin;
    SpinConfig_t config = Config(SpinMode_GIMBAL_FOLLOW);
    Check(spin.Init(&config), "modes init");
    spin.Set_WorldTarget(12.75f);
    spin.Set_theta(0.375f);
    spin.Set_MoveTarget(1.0f, -0.5f, 4.0f);
    for (int from = 0; from < 3; from++)
    {
        for (int to = 0; to < 3; to++)
        {
            spin.Set_Spin_Mode((SpinMode_e)from);
            spin.TIM_Calculate_PeriodElapsedCallback();
            spin.Set_Spin_Mode((SpinMode_e)to);
            spin.TIM_Calculate_PeriodElapsedCallback();
            SpinOutput_t out = spin.Get_Output();
            const float expected_w = 4.0f;
            Check(spin.Get_Spin_Mode() == to, "mode selection");
            Near(out.w, expected_w, 0.00001, "mode speed source");
            Near(out.forward, to == SpinMode_GIMBAL_FOLLOW ? 0.0 : -expected_w, 0.00001, "mode compensation");
            Near(out.x, cos(0.375) + 0.5 * sin(0.375), 0.00001, "mode keeps input frame");
            Near(out.y, sin(0.375) - 0.5 * cos(0.375), 0.00001, "mode keeps lateral input frame");
            Near(spin.Get_WorldTarget(), 12.75, 0.0, "mode preserves world target");
        }
    }
    spin.Set_Spin_Mode(SpinMode_GIMBAL_LOCK);
    spin.Set_MoveTarget(0.0f, 0.0f, -7.0f);
    spin.TIM_Calculate_PeriodElapsedCallback();
    Near(spin.Get_Output().forward, 7.0, 0.0, "negative spin compensation");
    SpinOutput_t copy = spin.Get_Output();
    copy.w = 123.0f;
    Check(copy.w != spin.Get_Output().w, "output returned by value");
    for (int mode = 0; mode < 3; mode++)
    {
        spin.Set_Spin_Mode((SpinMode_e)mode);
        spin.Set_MoveTarget(1.0f, -0.5f, -7.0f);
        spin.TIM_Calculate_PeriodElapsedCallback();
        const SpinOutput_t before = spin.Get_Output();
        const float world_target = -41.0f - mode * 8.0f;
        spin.Set_WorldTarget(world_target);
        spin.TIM_Calculate_PeriodElapsedCallback();
        const SpinOutput_t after = spin.Get_Output();
        Near(spin.Get_WorldTarget(), world_target, 0.0, "world target cache retains continuous angle");
        Check(before.x == after.x && before.y == after.y && before.w == after.w &&
              before.forward == after.forward, "world cache does not drive motion");
        spin.Set_theta(-0.9f);
        spin.TIM_Calculate_PeriodElapsedCallback();
        Near(spin.Get_Output().x, cos(-0.9) + 0.5 * sin(-0.9), 0.00001, "latest angle used without latch");
        Near(spin.Get_Output().y, sin(-0.9) - 0.5 * cos(-0.9), 0.00001, "latest lateral angle used without latch");
        Near(spin.Get_WorldTarget(), world_target, 0.0, "feedback never captures or overwrites world target");

        SpinMode fresh;
        config.mode = (SpinMode_e)mode;
        Check(fresh.Init(&config), "fresh history reference");
        fresh.Set_theta(-0.9f);
        fresh.Set_MoveTarget(1.0f, -0.5f, -7.0f);
        fresh.TIM_Calculate_PeriodElapsedCallback();
        const SpinOutput_t reference = fresh.Get_Output();
        const SpinOutput_t current = spin.Get_Output();
        Check(reference.x == current.x && reference.y == current.y && reference.w == current.w &&
              reference.forward == current.forward, "mode history does not change speed outputs");
    }
}

static void Lifecycle()
{
    SpinProbe first;
    SpinMode second;
    SpinConfig_t config = Config(SpinMode_CHASSIS_FOLLOW);
    Check(first.Init(&config), "first init");
    Check(second.Init(), "second init");
    first.Set_WorldTarget(3.5f);
    first.Set_theta(0.375f);
    first.Set_MoveTarget(1.0f, 2.0f, 8.0f);
    first.TIM_Calculate_PeriodElapsedCallback();
    first.Set_Spin_Mode(SpinMode_CHASSIS_FOLLOW);
    Check(first.Get_Count() == 1, "same mode preserves counter");
    first.TIM_Calculate_PeriodElapsedCallback();
    Check(first.Get_Count() == 2, "counter advances");
    first.Set_Spin_Mode((SpinMode_e)99);
    Check(first.Get_Spin_Mode() == SpinMode_CHASSIS_FOLLOW && first.Get_Count() == 2, "invalid mode preserves state");
    first.Set_Spin_Mode((SpinMode_e)3);
    Check(first.Get_Spin_Mode() == SpinMode_CHASSIS_FOLLOW && first.Get_Count() == 2, "removed fourth mode rejected");
    first.Set_Spin_Mode((SpinMode_e)-1);
    Check(first.Get_Spin_Mode() == SpinMode_CHASSIS_FOLLOW, "negative mode rejected");
    first.Set_Spin_Mode(SpinMode_GIMBAL_LOCK);
    Check(first.Get_Count() == 0, "new mode starts counter");
    second.TIM_Calculate_PeriodElapsedCallback();
    Near(second.Get_Output().x, 0.0, 0.0, "instance velocity isolation");
    Near(second.Get_Output().w, 0.0, 0.0, "instance angular input isolation");
    Near(second.Get_WorldTarget(), 0.0, 0.0, "instance world target isolation");
    Check(second.Get_Spin_Mode() == SpinMode_GIMBAL_FOLLOW, "instance mode isolation");

    SpinConfig_t invalid = config;
    invalid.zero_point = NAN;
    Check(!first.Init(&invalid), "reject invalid offset");
    invalid = config;
    invalid.mode = (SpinMode_e)99;
    Check(!first.Init(&invalid), "reject invalid initial mode");
    invalid.mode = (SpinMode_e)3;
    Check(!first.Init(&invalid), "reject removed fourth initial mode");
    Check(first.Get_Spin_Mode() == SpinMode_GIMBAL_LOCK, "rejected config preserves mode");
    Near(first.Get_WorldTarget(), 3.5, 0.0, "rejected config preserves target");

    config.zero_point = 0.7f;
    Check(first.Init(&config), "reinit");
    Check(first.Get_Count() == 0, "reinit resets counter");
    Near(first.Get_WorldTarget(), 0.0, 0.0, "reinit resets world target");
    SpinOutput_t reset = first.Get_Output();
    Check(reset.x == 0.0f && reset.y == 0.0f && reset.w == 0.0f && reset.forward == 0.0f, "reinit resets output");
    first.TIM_Calculate_PeriodElapsedCallback();
    Near(first.Get_Output().w, 0.0, 0.0, "reinit has no stale angular input");
    first.Set_MoveTarget(1.0f, 0.0f, 0.0f);
    first.TIM_Calculate_PeriodElapsedCallback();
    Near(first.Get_Output().x, 1.0, 0.00001, "neutral angle starts at configured zero");
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        return 2;
    }
    if (strcmp(argv[1], "transform") == 0) Transform();
    else if (strcmp(argv[1], "angular") == 0) Angular();
    else if (strcmp(argv[1], "modes") == 0) Modes();
    else if (strcmp(argv[1], "lifecycle") == 0) Lifecycle();
    else return 2;
    printf("%s: %d checks, %d failures\n", argv[1], checks, failures);
    return failures == 0 ? 0 : 1;
}
