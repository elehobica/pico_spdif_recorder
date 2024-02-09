# Raspberry Pi Pico spdif_recorder

## Overview
* Hi-Res WAV recorder from S/PDIF input to microSD card
* Audio format: 2ch, 16bit or 24bit
* Sampling frequencies: 44.1 KHz, 48.0 KHz, 88.2 KHz, 96.0 KHz (, 176.4 KHz, 192.0 KHz)
* Auto split detecting blank
* Manual split without gap

## Supported Board and Peripheral Devices
* Raspberry Pi Pico (rp2040)
* SPDIF Coaxial or TOSLINK Rx module (DLR1160 or equivalent)
* microSD cards (recommend SD-XC, V30 cards)

## Pin Assignment & Connection

![Circuit Diagram](doc/Pico_FatFs_Test_Schematic_wo_pullup.png)

### SPDIF Rx
| Pico Pin # | GPIO | Function | Connection |
----|----|----|----
| 20 | GP15 | DATA | from SPDIF data |

### microSD card

| Pico Pin # | Pin Name | Function | microSD connector | microSD SPI board |
----|----|----|----|----
|  4 | GP2 | SPI0_SCK | CLK (5) | CLK |
|  5 | GP3 | SPI0_TX | CMD (3) | MOSI |
|  6 | GP4 | SPI0_RX | DAT0 (7) | MISO |
|  7 | GP5 | SPI0_CSn | CD/DAT3 (2) | CS |
|  8 | GND | GND | VSS (6) | GND |
| 36 | 3V3(OUT) | 3.3V | VDD (4) | 3V3 |

Note:

* As for the wire length between Pico and SD card, short wiring as possible is desired, otherwise errors such as Mount error and Write fail will occur.

## How to build
* See ["Getting started with Raspberry Pi Pico"](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
* Put "pico-sdk", "pico-examples" and "pico-extras" on the same level with this project folder.
* Build is confirmed in Developer Command Prompt for VS 2022 and Visual Studio Code on Windows enviroment
* Confirmed with Pico SDK 1.5.1, cmake-3.27.2-windows-x86_64 and gcc-arm-none-eabi-10.3-2021.10-win32
```
> git clone -b 1.5.1 https://github.com/raspberrypi/pico-sdk.git
> cd pico-sdk
> git submodule update -i
> cd ..
> git clone -b sdk-1.5.1 https://github.com/raspberrypi/pico-examples.git
>
> git clone -b sdk-1.5.1 https://github.com/raspberrypi/pico-extras.git
> 
> git clone -b main https://github.com/elehobica/pico_spdif_recorder.git
```
* Lanuch "Developer Command Prompt for VS 2019"
```
> cd pico_spdif_recorder
> mkdir build && cd build
> cmake -G "NMake Makefiles" ..
> nmake
```
* Put "pico_spdif_recorder.uf2" on RPI-RP2 drive

## Serial command interface
* Serial interface is available from USB port of Raspberry Pi Pico.

| Key | Function |
----|----
| Space | Recording Standby / Recording Stop |
| s | Immediate split |
| r | Resolution change 16bit/24bit (default: 16bit) |
| b | Blank split on/off (default: on) |
| v | Verbose for monitoring messages |
| c | Clear WAV file suffix to 1 |
| h | Help |

