#include "vtm_legacy.h"
#include "bsp_uart.h"
#include "crc_ref.h"
#include "referee_daemon.h"

#include <string.h>

#define VTM_LEGACY_BUFFER_SIZE 255U
#define VTM_LEGACY_HEADER_SIZE 5U
#define VTM_LEGACY_FRAME_OVERHEAD 9U
#define VTM_LEGACY_KEYBOARD_ID 0x0304U
#define VTM_LEGACY_KEYBOARD_SIZE 12U

static UART_HandleTypeDef *vtm_uart;
static uint8_t stream[VTM_LEGACY_BUFFER_SIZE];
static uint16_t stream_used;
static Struct_VTM_Legacy_Keyboard_Snapshot keyboard_snapshot;
static bool keyboard_received;
static bool has_valid_frame;

static uint16_t DecodeLittleU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void DecodeKeyboard(const uint8_t *frame)
{
    const uint8_t *data = frame + 7U;
    if (data[6] > 1U || data[7] > 1U)
        return;

    Struct_VTM_Legacy_Keyboard_Snapshot next = {0};
    next.mouse_x = (int16_t)DecodeLittleU16(data);
    next.mouse_y = (int16_t)DecodeLittleU16(data + 2U);
    next.mouse_z = (int16_t)DecodeLittleU16(data + 4U);
    next.mouse_left = data[6];
    next.mouse_right = data[7];
    next.keyboard = DecodeLittleU16(data + 8U);
    next.frame_sequence = frame[3];
    next.received_ms = HAL_GetTick();
    next.sequence = keyboard_snapshot.sequence + 1U;
    keyboard_snapshot = next;
    keyboard_received = true;
}

/** 有界跨 DMA chunk 拼帧；坏帧跳过一字节，以保留后续可能的帧头。 */
void VTM_Legacy_ReceiveData(uint8_t *data, uint16_t length)
{
    if (data == NULL)
        return;
    for (uint16_t i = 0U; i < length; ++i)
    {
        stream[stream_used++] = data[i];
        while (stream_used > 0U)
        {
            uint16_t expected = 0U;
            if (stream[0] == 0xA5U)
            {
                if (stream_used < VTM_LEGACY_HEADER_SIZE)
                    break;
                if (Verify_CRC8_Check_Sum(stream, VTM_LEGACY_HEADER_SIZE))
                {
                    const uint32_t total = (uint32_t)DecodeLittleU16(stream + 1U) +
                                           VTM_LEGACY_FRAME_OVERHEAD;
                    if (total <= VTM_LEGACY_BUFFER_SIZE)
                        expected = (uint16_t)total;
                }
            }
            if (expected != 0U && stream_used < expected)
                break;

            uint16_t consumed = 1U;
            if (expected != 0U && Verify_CRC16_Check_Sum(stream, expected))
            {
                if (DecodeLittleU16(stream + 5U) == VTM_LEGACY_KEYBOARD_ID &&
                    expected == VTM_LEGACY_KEYBOARD_SIZE + VTM_LEGACY_FRAME_OVERHEAD)
                    DecodeKeyboard(stream);
                has_valid_frame = true;
                if (vtm_uart != NULL)
                    VTMDaemonFeed();
                consumed = expected;
            }
            stream_used -= consumed;
            memmove(stream, stream + consumed, stream_used);
        }
    }
}

bool VTM_Legacy_Init(UART_HandleTypeDef *uart)
{
    if (vtm_uart != NULL)
        return vtm_uart == uart;
    if (uart == NULL || !VTMDaemonRegister())
        return false;
    stream_used = 0U;
    keyboard_received = false;
    has_valid_frame = false;
    memset(&keyboard_snapshot, 0, sizeof(keyboard_snapshot));
    vtm_uart = uart;
    UART_Init(uart, VTM_Legacy_ReceiveData);
    return true;
}

bool VTM_Legacy_ReadKeyboardSnapshot(Struct_VTM_Legacy_Keyboard_Snapshot *snapshot)
{
    if (snapshot == NULL)
        return false;
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool valid = keyboard_received;
    if (valid)
        *snapshot = keyboard_snapshot;
    __DMB();
    __set_PRIMASK(mask);
    return valid;
}

bool VTM_Legacy_IsOnline(void)
{
    return vtm_uart != NULL && has_valid_frame && VTMDaemonIsOnline();
}
