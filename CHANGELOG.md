# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]
### Added
* Add adaptive blank level detection
* Add Raspberry Pi Pico 2 series boards
* Add workflow to build binaries
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