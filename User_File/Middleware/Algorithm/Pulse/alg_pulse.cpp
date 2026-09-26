#include "alg_pulse.h"

void Pulse_Dispatch(const PulseEntry_t *entries, size_t entry_count, uint32_t tick_ms)
{
    for (size_t index = 0; index < entry_count; index++)
    {
        if (tick_ms % entries[index].period_ms == 0)
        {
            entries[index].callback();
        }
    }
}
