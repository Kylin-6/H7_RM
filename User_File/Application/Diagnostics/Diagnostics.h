#ifndef CHASSIS_DIAGNOSTICS_H
#define CHASSIS_DIAGNOSTICS_H
#include "Chassis.h"
#include "message_types.h"

struct Struct_Diagnostic_Pattern
{
    uint32_t selected_fault = 0U;
    uint8_t red = 0U, green = 0U, blue = 0U;
    uint8_t pulses = 0U;
    bool slow = false;
};
struct Struct_Diagnostic_LED_State
{
    uint32_t fault_mask = 0U;
    Struct_Diagnostic_Pattern pattern{};
};
/** TIM_1ms_Task 唯一写入，仅供调试器观察。 */
extern Struct_Diagnostic_LED_State Diagnostics_LED_State;
Struct_Chassis_Diagnostic Diagnostics_BuildSnapshot(const Struct_Chassis_Diagnostic_Input &chassis,
                                                    bool remote_valid, const bool enable_timeout[5]);
Struct_Diagnostic_Pattern Diagnostics_SelectPattern(const Struct_Chassis_Diagnostic &snapshot,
                                                    bool system_fatal, bool control_fresh,
                                                    uint32_t uptime_ms, uint32_t &visible_mask);
bool Diagnostics_LightOn(const Struct_Diagnostic_Pattern &pattern, uint32_t elapsed_ms);
/** ControlTask 在应用更新后每 10 ms 调用。 */
void Diagnostics_Publish(void);
void Diagnostics_PublishInitFailure(void);
/** TIM_1ms_Task 选择灯效、设置颜色并刷新 SPI6。 */
void Diagnostics_LED_Update(void);
#endif
