#ifndef VTM_LEGACY_H
#define VTM_LEGACY_H

#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

/** VT02/VT12 的 A5 / 0x0304 键鼠数据，不含遥控摇杆、挡位或暂停字段。 */
typedef struct
{
    int16_t mouse_x, mouse_y, mouse_z;
    uint8_t mouse_left, mouse_right;
    uint16_t keyboard;
    uint8_t frame_sequence;
    uint32_t received_ms, sequence;
} Struct_VTM_Legacy_Keyboard_Snapshot;

#ifdef __cplusplus
extern "C" {
#endif

/** UART 参数由板级初始化设置；同一串口只能选择一个图传解析器。 */
bool VTM_Legacy_Init(UART_HandleTypeDef *uart);
/** 仅由 UART 回调写入；测试入口不可与 UART 中断并发调用。 */
void VTM_Legacy_ReceiveData(uint8_t *data, uint16_t length);
/** 首次收到合法 0x0304 才返回 true；时效由 Input 按 received_ms 检查。 */
bool VTM_Legacy_ReadKeyboardSnapshot(Struct_VTM_Legacy_Keyboard_Snapshot *snapshot);
/** 链路诊断包含合法普通图传帧，不能替代键鼠业务时效。 */
bool VTM_Legacy_IsOnline(void);

#ifdef __cplusplus
}
#endif

#endif
