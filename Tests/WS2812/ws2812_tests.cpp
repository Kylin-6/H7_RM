#include "bsp_ws2812.h"
#undef NDEBUG
#include <assert.h>
#include <string.h>

static SPI_HandleTypeDef spi6;
Struct_SPI_Manage_Object SPI6_Manage_Object{&spi6};
static unsigned sends;
static bool fail_next;
static uint8_t last_frame[25];

uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *handler, GPIO_TypeDef *gpio,
                          uint16_t pin, GPIO_PinState level,
                          const uint8_t *data, uint16_t length)
{
    assert(handler == &spi6 && gpio == nullptr && pin == 0);
    assert(level == GPIO_PIN_SET && length == sizeof(last_frame));
    memcpy(last_frame, data, sizeof(last_frame));
    ++sends;
    if (fail_next)
    {
        fail_next = false;
        return HAL_ERROR;
    }
    return HAL_OK;
}

int main()
{
    Class_WS2812 led;
    led.Init();
    led.TIM_10ms_Write_PeriodElapsedCallback();
    assert(sends == 1);
    for (int i = 0; i < 24; ++i)
        assert(last_frame[i] == 0x60);
    assert(last_frame[24] == 0);
    for (int i = 0; i < 1000; ++i)
        led.TIM_10ms_Write_PeriodElapsedCallback();
    assert(sends == 1);

    led.Set_RGB(255, 0, 0);
    led.TIM_10ms_Write_PeriodElapsedCallback();
    assert(sends == 2);
    for (int i = 8; i < 16; ++i)
        assert(last_frame[i] == 0x78);
    led.Set_RGB(255, 0, 0);
    led.TIM_10ms_Write_PeriodElapsedCallback();
    assert(sends == 2);

    led.Set_RGB(0, 0, 255);
    fail_next = true;
    led.TIM_10ms_Write_PeriodElapsedCallback();
    assert(sends == 3);
    led.TIM_10ms_Write_PeriodElapsedCallback();
    assert(sends == 4);
    led.TIM_10ms_Write_PeriodElapsedCallback();
    assert(sends == 4);

    led.Init(0, 255, 0);
    led.TIM_10ms_Write_PeriodElapsedCallback();
    assert(sends == 5);
    return 0;
}
