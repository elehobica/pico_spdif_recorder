# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]
### Added
* Add periodic `f_sync` of WAV file every 64 MB on core0 to write exFAT FAT chain gradually instead of at once in finalize (configurable by `PERIODIC_SYNC_MBYTES` or `-s` option of `build_docker.sh`, 0 to disable)
### Fixed
* Fix audio gap at the beginning of next file after splitting large WAV file (e.g. at 4 GB limit) caused by long FATFS blocking in finalize
* Fix audio gap and pre-roll corruption at recording start by preparing the first WAV file at standby start
* Fix `%` in log messages on serial output
* Fix FATFS work area declared as local variable of mount function, which kept being referred to after mount
* Fix WAV data size not aligned to sample frame when trailing blank is truncated

## [1.0.7] - 2026-10-17
### Changed
* Support pico-sdk 2.3.0
* Support persistence for auto blank split function as `CFG_AUTO_BLANK_SPLIT`
* Move firmware version from flash parameter to constant `APP_VERSION` in `src/version.h`
### Fixed
* Reject Wi-Fi configuration command while recording to avoid flash programming during recording

## [1.0.6] - 2026-06-08
### Added
* Add `build_docker.sh` for Docker-based local build (mirrors CI workflow)
* Add Wi-Fi / NTP connecting heartbeat LED pattern on Pico W / Pico 2 W (50 ms pulse per second)
### Changed
* Stand-by LED is now steady ON (previously OFF)
* Refactor Wi-Fi connect to async polling so LED can be driven safely from the main thread
### Fixed
* Avoid redundant Wi-Fi connect retries on successful connection

## [1.0.5] - 2026-03-21
### Added
* Add wav_tagger environment as a helper tool for generated wav file post-processing
### Changed
* Replace set-version action with composite script
### Fixed
* Revise PIN_SWITCH_24BIT switch capture timing to avoid initial mis-judgment

## [1.0.4] - 2025-06-14
### Added
* Add adaptive blank level detection
* Add Raspberry Pi Pico 2 series boards
* Add GiHub Actions for build and release
### Changed
* Use pico_flash_param submodule
* Support pico-sdk 2.1.1 (previously 1.5.1)
* Log only when recording done on log file
### Fixed
* Add workaround for mount fail case of Samsung PRO Plus card

## [1.0.1] - 2024-03-30
### Added
* Support Raspberry Pi Pico W to reflect timestamp from NTP
* Store Wi-Fi configuration in user flash (Pico W only)
* Show errors by LED blink
* Have a little blank time before emerging sound at start (PRE_START_SEC)
### Changed
* Introduce FatFs R0.15 (previously R0.14b)
* Confirm bandwidth situation and revise card recommendation (Samsung PRO Plus 256GB is the best currently)
### Fixed
* Fix bug for seek position in case of large size file when closing file

## [1.0.0] - 2024-02-12
* Initial release