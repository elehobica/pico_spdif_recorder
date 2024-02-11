# Raspberry Pi Pico spdif_recorder

![PCB Front](doc/pico_spdif_recorder_pcb_front.jpg)

## Overview
* Hi-Res recorder from S/PDIF input to WAV files on microSD card
* Bit resolution: 16bit or 24bit (2ch)
* Sampling frequency: 44.1 KHz, 48.0 KHz, 88.2 KHz, 96.0 KHz
* Recording only, no monitoring or playback functions

## Supported Board and Peripheral Devices
* Raspberry Pi Pico (rp2040)
* S/PDIF Coaxial or TOSLINK Rx module (DLR1160 or equivalent)
* microSD cards (recommend SD-XC, V30 cards)

## Pin Assignment & Connection

![Circuit Diagram](doc/pico_spdif_recorder_circuit.png)

### S/PDIF Rx
| Pico Pin # | GPIO | Function | Connection |
----|----|----|----
| 20 | GP15 | DATA | from S/PDIF data |

### microSD card

| Pico Pin # | Pin Name | Function | microSD Pin Name | microSD Pin # |
----|----|----|----|----
|  4 | GP2 | SPI0_SCK | CLK | 5 |
|  5 | GP3 | SPI0_TX | CMD | 3 |
|  6 | GP4 | SPI0_RX | DAT0 | 7 |
|  7 | GP5 | SPI0_CSn | CD/DAT3 | 2 |
|  8 | GND | GND | VSS | 6 |
| 36 | 3V3(OUT) | 3.3V | VDD | 4 |

Note:
* As for the wire length between Pico and SD card, short wiring as possible is desired, otherwise errors such as Mount error and Write fail will occur.

### Button/Switch (Optional)
| Pico Pin # | GPIO | Function | Connection |
----|----|----|----
| 9 | GP6 | Input | 16bit/24bit switch |
| 10 | GP7 | Input | Start/Stop button |

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
| Space | Recording Start Standby / Recording Stop |
| r | Resolution change 16bit/24bit (default: 16bit) |
| s | Manual split |
| b | Auto blank split on/off (default: on) |
| v | Verbose for monitoring messages |
| c | Clear WAV file suffix to 1 |
| h | Help |

### Recording start/stop/split features
* Standby recording start while silence and auto start when sound detected
* Auto stop and standby restart when long blank detected
* Auto split of WAV file when short blank detected
* Manual immediate split of WAV file without gap

### LED indicator
* Slow blink: Recording is on-going.
* Fast blink: Background file proceses to close previous file and prepare next file are on-going. During this term, no command requests can be accepted.

## microSD card recommendation
* Due to the limitation of single bit SPI interface driven by Raspberry Pi Pico, even with highest class microSD cards (as of 2024), recording in 24bit 176.4 KHz or 192.0 KHz is quite challenging. It will sometimes causes the drops of audio sampling data. The bandwidth status can be monitored in Verbose mode.
* With following recommended microSD cards, recording in 24bit 96.0 KHz will be stable as far as experimentally confirmed.
* Format micorSD card in exFAT with [official SD Card Formatter](https://www.sdcard.org/downloads/formatter/) before usage. 

| Vendor | Product Name | Part Number |
----|----|----
| SanDisk | Extreme PRO 256GB | SDSQXCD-256G-GN6MA |
| Samsung | PRO Plus 256GB | MB-MD256SA |

<img src="doc/sandisk-extreme-pro-256gb.jpg" width="80" />  <img src="doc/samsung-pro-plus-256gb.jpg" width="80" />
