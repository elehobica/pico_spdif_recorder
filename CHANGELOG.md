# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]
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