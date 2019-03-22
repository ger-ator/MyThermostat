# MyThermostat

MyThermostat is a drop-in replacement board for Farho Xana Plus oil heater.

Provided sketch is a draft thermostat node for MySensors network.

## Features
- ATMega328P makes it Arduino compatible.
- RFM69 for extended range.
- ATSHA204 for hardware signing.
- AT25DF512C FOTA programming enabled.
- Onboard DS18B20 for temperature protection purposes.
- 0,96" Oled display.
- Linear power supply, so no pitch noise around bedroom.
- Power SSR with built-in heatsink. No noise switching.
- Uses the original 2x2 keypad for user interaction.

The board is exactly the same form factor as the factory one so it can be mounted without modifying the original frame. Just unplug the old board and drop-in the new one. The goal is to keep heater modifications as low as possible.

![MyThermostat board](https://github.com/ger-ator/MyThermostat/raw/master/pics/v2_1.jpg)
