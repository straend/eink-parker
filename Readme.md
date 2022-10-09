## Eink Parker
Displays Your parking time (next whole or half hour) with a seven-segment-font on 
an e-ink display. Keeps time with a ds1302 realtimeclock.

## Hardware

[Arch BLE](https://wiki.seeedstudio.com/Arch_BLE/) any platform with embedded hal bindings should work (needs I2C and SPI)
[2.7inch eink](https://wiki.seeedstudio.com/2.7inch-Triple-Color-E-Ink-Shield-for-Arduino/)
Buttons on shield are used for setting time.

OLED display (ssd1306), only used for setting time
DS1302 or another RTC chip with battery backup


#### Flashing
Using [cargo-embed](https://probe.rs/docs/tools/cargo-embed/)

    cargo embed --release

#### To set time
Press both buttons on e-ink shield, OLED-screen will show settings.
btn2 increases seconds/minutes/hours
btn1 cycles between seconds/minutes/hours

