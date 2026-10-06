#include "ChassisFlashLog.h"

#include "bsp_w25q64jv.h"

#include <cstring>

volatile Struct_Chassis_Flash_Debug ChassisFlashLog_Debug{};
__attribute__((section(".ram_d1_data"), aligned(32)))
uint8_t ChassisFlashLog_ReadBuffer[4096];

namespace
{
constexpr uint32_t kMagic = 0x31474c43U; // CLG1
constexpr uint32_t kQueueSize = 256U;
struct Page
{
    uint32_t magic;
    uint16_t count;
    uint16_t record_size;
    uint32_t index;
    uint32_t crc;
    Struct_Chassis_Flash_Record record[2];
};
static_assert(sizeof(Page) == 256U, "Flash page format");
// NOLOAD 区无需清零：只读取已发布的 head/tail 范围。
__attribute__((section(".ram_d1_data"), aligned(32)))
Struct_Chassis_Flash_Record queue[kQueueSize];
volatile uint32_t head = 0U;
volatile uint32_t tail = 0U;
volatile bool accepting = true; // Flash 就绪前先保留控制任务的开机样本。
uint32_t sequence = 0U;

bool Blank(const void* data, uint32_t size)
{
    const auto* bytes = static_cast<const uint8_t*>(data);
    for (uint32_t i = 0U; i < size; ++i)
        if (bytes[i] != 0xffU)
            return false;
    return true;
}

uint32_t Crc(const void* data, uint32_t size)
{
    uint32_t crc = 0xffffffffU;
    const auto* bytes = static_cast<const uint8_t*>(data);
    for (uint32_t i = 0U; i < size; ++i)
    {
        crc ^= bytes[i];
        for (uint32_t bit = 0U; bit < 8U; ++bit)
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xedb88320U : 0U);
    }
    return ~crc;
}

bool Pop(Struct_Chassis_Flash_Record& record)
{
    const uint32_t read = tail;
    if (read == head)
        return false;
    __DMB();
    record = queue[read % kQueueSize];
    __DMB();
    tail = read + 1U;
    return true;
}
} // namespace

void ChassisFlashLog_Capture(const Struct_Chassis_Flash_Record& record)
{
    if (!accepting)
        return;
    const uint32_t write = head;
    const uint32_t sample_sequence = sequence++;
    if (write - tail >= kQueueSize)
    {
        ++ChassisFlashLog_Debug.dropped_records;
        return;
    }
    __DMB();
    queue[write % kQueueSize] = record;
    queue[write % kQueueSize].sequence = sample_sequence;
    __DMB();
    head = write + 1U;
}

void ChassisFlashLog_Run()
{
    auto& debug = ChassisFlashLog_Debug;
    if (!BSP_W25Q64JV.Is_Initialized())
    {
        accepting = false;
        debug.state = 4U;
        osThreadExit();
        return;
    }
    if (!BSP_W25Q64JV.Enable_Quad_Mode())
    {
        accepting = false;
        debug.state = 4U;
        osThreadExit();
        return;
    }

    Page page;
    // 只追加到空白页；历史日志（包括掉电时的半页）保留，不执行擦除。
    while (debug.next_address < W25Q64JV_FLASH_SIZE)
    {
        if (!BSP_W25Q64JV.Read_Data(&page, static_cast<uint32_t>(debug.next_address), sizeof(page)))
        {
            debug.state = 4U;
            break;
        }
        if (Blank(&page, sizeof(page)))
            break;
        if (page.magic != kMagic || page.record_size != sizeof(page.record[0]))
        {
            debug.state = 5U;
            break;
        }
        debug.next_address += sizeof(page);
    }
    if (debug.state == 0U)
    {
        debug.state = debug.next_address == W25Q64JV_FLASH_SIZE ? 3U : 1U;
    }
    if (debug.state != 1U)
        accepting = false;

    for (;;)
    {
        const uint32_t command = debug.command;
        if (command == 1U)
            accepting = false;
        if ((debug.state == 1U || debug.state == 2U) &&
            (head - tail >= 2U || (command == 1U && head != tail)))
        {
            std::memset(&page, 0xff, sizeof(page));
            page.magic = kMagic;
            page.record_size = sizeof(page.record[0]);
            page.index = debug.next_address / sizeof(page);
            page.count = Pop(page.record[0]) ? 1U : 0U;
            if (Pop(page.record[1]))
                ++page.count;
            page.crc = Crc(page.record, sizeof(page.record));
            // 每页写前确认空白、写后回读确认，保护历史数据且发现保护位/写失败。
            Page verify;
            if (!BSP_W25Q64JV.Read_Data(&verify, static_cast<uint32_t>(debug.next_address), sizeof(verify)) ||
                !Blank(&verify, sizeof(verify)))
            {
                debug.state = 5U;
                accepting = false;
            }
            else if (!BSP_W25Q64JV.Write_Data(&page, static_cast<uint32_t>(debug.next_address), sizeof(page)) ||
                     !BSP_W25Q64JV.Read_Data(&verify, static_cast<uint32_t>(debug.next_address), sizeof(verify)) ||
                     std::memcmp(&page, &verify, sizeof(page)) != 0)
            {
                debug.state = 4U;
                accepting = false;
            }
            else
            {
                debug.written_records += page.count;
                debug.next_address += sizeof(page);
                debug.state = debug.next_address == W25Q64JV_FLASH_SIZE ? 3U : 2U;
                if (debug.state == 3U)
                    accepting = false;
            }
        }
        else if (command == 1U)
        {
            if (debug.state == 1U || debug.state == 2U)
                debug.state = 6U;
            __DMB();
            debug.command = 0U;
        }
        else if (command == 2U)
        {
            const uint32_t address = debug.read_address;
            const uint32_t length = debug.read_length;
            debug.read_result = 2U;
            if (!accepting && length > 0U && length <= sizeof(ChassisFlashLog_ReadBuffer) &&
                address < W25Q64JV_FLASH_SIZE && length <= W25Q64JV_FLASH_SIZE - address)
                debug.read_result = BSP_W25Q64JV.Read_Data(ChassisFlashLog_ReadBuffer, address, length) ? 0U : 1U;
            __DMB();
            debug.command = 0U;
        }
        osDelay(1U);
    }
}
