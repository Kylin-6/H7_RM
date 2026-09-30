#pragma once
struct BoardHardware { bool imu, indicators, flash, adc, power, usb_debug; };
inline const BoardHardware &BoardConfig_Get()
{
    static const BoardHardware hardware{true,true,true,true,true,true};
    return hardware;
}
