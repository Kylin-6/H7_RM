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

// 曲谱来源：NCUROBOT (C) 2022，Buzzer-YOU/BSP/buzzer.c 的 gala_you()。
// GALA《Young For You》，保留 Note() 的原始音高及 Long * 200 ms 时值。
constexpr Struct_Score_Note score[] = {
    {494.0f, 200U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
    {494.0f, 400U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
    {494.0f, 400U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
    {587.0f, 400U, 0U}, // note_5D
    {392.0f, 200U, 0U}, // note_G
    {523.0f, 200U, 0U}, // note_5C
    {523.0f, 200U, 0U}, // note_5C
    {392.0f, 200U, 0U}, // note_G
    {494.0f, 200U, 0U}, // note_5B
    {523.0f, 200U, 0U}, // note_5C
    {494.0f, 200U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
    {494.0f, 400U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
    {494.0f, 400U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
    {587.0f, 400U, 0U}, // note_5D
    {392.0f, 200U, 0U}, // note_G
    {523.0f, 200U, 0U}, // note_5C
    {523.0f, 200U, 0U}, // note_5C
    {392.0f, 200U, 0U}, // note_G
    {494.0f, 200U, 0U}, // note_5B
    {523.0f, 200U, 0U}, // note_5C
    {494.0f, 200U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
    {494.0f, 400U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
    {494.0f, 400U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
    {587.0f, 400U, 0U}, // note_5D
    {392.0f, 200U, 0U}, // note_G
    {523.0f, 200U, 0U}, // note_5C
    {523.0f, 200U, 0U}, // note_5C
    {392.0f, 200U, 0U}, // note_G
    {494.0f, 200U, 0U}, // note_5B
    {523.0f, 200U, 0U}, // note_5C
    {494.0f, 200U, 0U}, // note_5B
    {392.0f, 200U, 0U}, // note_G
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
        if (score[index].frequency_hz > 0.0f)
        {
            BSP_Buzzer.Set_Frequency(score[index].frequency_hz);
            BSP_Buzzer.Set_Loudness(1.0f);
        }
        else
            BSP_Buzzer.Set_Loudness(0.0f);
    }
}
