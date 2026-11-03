# AVR-DB Firmware

Combined firmware for the EmonTx4, EmonPi2 and EmonTx5

## [emon_DB_6CT](emon_DB_6CT)

EmonLibDB based continuous monitoring firmware for the EmonTx4, EmonPi2 and EmonTx5. Supports:

- 1-phase or 3-phase voltage, selectable at compilation.
- 6 CT current sensors, real power & cumulative energy values reported.
- Pulse counting (default's to pulse on analog input).
- Serial configuration of radio settings (EmonTx4/5).
- Serial configuration of voltage and current sensor calibration.
- Option to transmit data over RFM69CW radio (EmonTx4/5).
- Optional analog input.
- Serial output for direct serial connection (emonPi2) or USB connection (emonTx4/emonTx5).
- emonPi2 OLED starting message

*DS18B20 temperature sensing is not supported on the AVR-DB core, temperature sensing is handled by the RaspberryPi directly on the EmonPi2.* 

## [emon_CM_6CT_temperature](emon_CM_6CT_temperature)

EmonLibCM based continuous monitoring firmware for the EmonTx4, EmonPi2 and EmonTx5. Supports:

- 1-phase voltage only (or no voltage sensing, apparent power or current only)
- 6 CT current sensors, real power & cumulative energy values reported.
- Pulse counting (default's to pulse on analog input).
- Serial configuration of radio settings (EmonTx4/5).
- Serial configuration of voltage and current sensor calibration.
- Option to transmit data over RFM69CW radio (EmonTx4/5).
- Optional analog input.
- Serial output for direct serial connection (emonPi2) or USB connection (emonTx4/emonTx5).
- emonPi2 OLED starting message
- DS18B20 Temperature sensing support on the AVR-DB core.

## [emon_DB_12CT](emon_DB_12CT)

EmonLibDB based continuous monitoring firmware for the EmonTx4, EmonPi2 and EmonTx5. Supports:

- 1-phase or 3-phase voltage, selectable at compilation.
- 12 CT current sensors, real power & cumulative energy values reported.
- Option to transmit data over RFM69CW radio (EmonTx4/5).
- Serial output for direct serial connection (emonPi2) or USB connection (emonTx4/emonTx5).
- Serial configuration of radio settings (EmonTx4/5).
- Serial configuration of voltage and current sensor calibration.
- emonPi2 OLED starting message

*DS18B20 temperature sensing and serial configuration is not supported.*

## Building

All pre-compiled firmware variants can be built with [PlatformIO](https://platformio.org/):

    ./build.sh                     # build everything into compiled/
    ./build.sh emon_DB_12CT        # build one firmware
    ./build.sh -o /tmp/hex         # write hex files somewhere else

Each variant is an `[env:...]` in the firmware's `platformio.ini` that overrides the configuration `#define`s at the top of the `.ino` with build flags. The env name becomes the hex file name, with the version taken from `firmware_version`, e.g. `compiled/emonTx4/emonTx4_DB_12CT_1phase_LPL_v1_3_1.ino.hex`. To add a variant, add an env.

For development, `pio run -t upload` in a firmware directory builds and uploads using the settings in the `.ino` source.
