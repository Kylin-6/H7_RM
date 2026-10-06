#include "BuzzerMusic.h"
#include "bsp_buzzer.h"
#include "board_config.h"
#include "Init.h"
#include <cstddef>

extern volatile bool init_finished;

namespace
{
struct Struct_Score_Note
{
    float frequency_hz;
    uint16_t sound_ms;
    uint16_t gap_ms;
};

// 转录自用户提供的 remember_mr_zhang_score.c，保留音高与毫秒时值。
constexpr Struct_Score_Note score[] = {
    {698.456f, 198U, 2U}, // F5
    {587.330f, 198U, 2U}, // D5
    {698.456f, 198U, 2U}, // F5
    {587.330f, 198U, 2U}, // D5
    {1046.502f, 798U, 2U}, // C6
    {932.328f, 198U, 2U}, // AS5
    {932.328f, 198U, 2U}, // AS5
    {932.328f, 198U, 2U}, // AS5
    {587.330f, 98U, 2U}, // D5
    {698.456f, 500U, 400U}, // F5
    {783.991f, 398U, 2U}, // G5
    {783.991f, 198U, 2U}, // G5
    {783.991f, 98U, 2U}, // G5
    {698.456f, 498U, 2U}, // F5
    {587.330f, 198U, 2U}, // D5
    {466.164f, 198U, 2U}, // AS4
    {622.254f, 198U, 2U}, // DS5
    {587.330f, 198U, 2U}, // D5
    {622.254f, 98U, 2U}, // DS5
    {783.991f, 298U, 2U}, // G5
    {698.456f, 798U, 2U}, // F5
    {698.456f, 198U, 2U}, // F5
    {587.330f, 198U, 2U}, // D5
    {698.456f, 198U, 2U}, // F5
    {587.330f, 198U, 2U}, // D5
    {1046.502f, 798U, 2U}, // C6
    {932.328f, 198U, 2U}, // AS5
    {932.328f, 198U, 2U}, // AS5
    {932.328f, 198U, 2U}, // AS5
    {587.330f, 98U, 2U}, // D5
    {783.991f, 98U, 2U}, // G5
    {698.456f, 400U, 200U}, // F5
    {783.991f, 98U, 2U}, // G5
    {880.000f, 98U, 2U}, // A5
    {932.328f, 398U, 2U}, // AS5
    {932.328f, 198U, 2U}, // AS5
    {698.456f, 198U, 2U}, // F5
    {698.456f, 198U, 2U}, // F5
    {932.328f, 398U, 2U}, // AS5
    {587.330f, 198U, 2U}, // D5
    {523.251f, 198U, 2U}, // C5
    {587.330f, 398U, 2U}, // D5
    {523.251f, 198U, 2U}, // C5
    {466.164f, 800U, 0U}, // AS4
};
}

void BuzzerMusic_Update(void)
{
    if (!init_finished || System_Init_GetState() == SYSTEM_INIT_FATAL ||
        !BoardConfig_Get().indicators) return;
    static size_t index = 0U;
    static uint32_t started_ms = 0U;
    static uint32_t playback_started_ms = 0U;
    static bool playback_started = false;
    static bool started = false;
    static bool in_gap = false;
    if (index >= sizeof(score) / sizeof(score[0])) return;
    const uint32_t now = HAL_GetTick();
    if (!playback_started)
    {
        playback_started_ms = now;
        playback_started = true;
    }
    if (now - playback_started_ms >= 10000U)
    {
        BSP_Buzzer.Set_Loudness(0.0f);
        index = sizeof(score) / sizeof(score[0]);
        return;
    }
    const uint32_t elapsed = now - started_ms;
    if (started && !in_gap && elapsed >= score[index].sound_ms)
    {
        BSP_Buzzer.Set_Loudness(0.0f);
        in_gap = true;
    }
    if (started && elapsed >= static_cast<uint32_t>(score[index].sound_ms) + score[index].gap_ms)
    {
        ++index;
        started = false;
        in_gap = false;
    }
    if (!started && index < sizeof(score) / sizeof(score[0]))
    {
        started_ms = now;
        started = true;
        BSP_Buzzer.Set_Frequency(score[index].frequency_hz);
        BSP_Buzzer.Set_Loudness(0.15f);
    }
}
