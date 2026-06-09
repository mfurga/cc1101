# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0/).

## [1.3.0] - 2026-06-09

### Added
- `STATUS_TIMEOUT` status code, returned by `receive()` and `transmit()` on timeout.

### Fixed
- `receive()` and `transmit()` no longer hang, and handle the RX/TX FIFO as the errata (SWRZ020E) requires.
- `getRSSI()` returned wrong values for raw readings of 128 or more.
- Frequency deviation, channel spacing and RX bandwidth ranges are now validated correctly.
- OOK/ASK reception uses the AGC settings from DN022; the FSK defaults are not optimal for OOK.
- Avoid a `digitalRead()` warning on ESP32 with a custom MISO pin.

## [1.2.2] - 2026-02-14

### Changed
- Excluded the `assets` folder from the published library package.

## [1.2.1] - 2026-02-13

### Added
- Made the register access methods (`readReg`, `writeReg`, `readRegBurst`, etc.) public for direct configuration.
- Added the missing status register defines.

## [1.2.0] - 2025-08-15

### Added
- Data whitening support.
- Forward Error Correction (FEC) support.
- Manchester encoding support.

## [1.1.5] - 2025-08-14

### Fixed
- Added `yield()` calls during long waits so the board's watchdog does not reset it (for example on ESP boards).

## [1.1.4] - 2025-08-12

### Fixed
- Corrected a wrong preamble length value (36 changed to 32).

## [1.1.3] - 2025-07-17

### Added
- Handle the TXFIFO_UNDERFLOW and RXFIFO_OVERFLOW states.

### Changed
- Wrapped static strings in the `F()` macro in the examples to save RAM.

## [1.1.2] - 2025-05-13

### Added
- ESP32-S3 support.

### Fixed
- Renamed `cc1101.cc` to `cc1101.cpp` so the Arduino IDE compiles the library.

## [1.1.1] - 2025-05-11

### Added
- ESP32 and ESP32-C3 support.

### Fixed
- Fixed a FIFO overflow.

## [1.1.0] - 2025-05-10

### Added
- A `size` parameter to `receive()` so the caller can set the buffer size.

## [1.0.5] - 2025-05-07

### Added
- A "Hello World" transmit and receive example.

## [1.0.4] - 2025-05-07

### Fixed
- Recognize CC1101 chips that report chip version `0x04`, so `begin()` no longer fails for them.

## [1.0.3] - 2025-02-17

### Fixed
- Fixed the frequency deviation, channel spacing and data rate calculations, which used integer math and could return wrong values (now done in floating-point).

## [1.0.2] - 2025-01-28

### Fixed
- Fixed wrong byte counts in `transmit()` and `receive()` caused by mismatched integer types.

## [1.0.1] - 2025-01-15

### Added
- Initial release: send and receive packets with the CC1101.
- Modulation formats: 2-FSK, GFSK, ASK/OOK, 4-FSK and MSK.
- Set the frequency, data rate, frequency deviation, channel and RX bandwidth.
- Packet options: fixed and variable length, address filtering, CRC and sync word.
- Output power control and RSSI/LQI readout.
- Blocking receive and a receive callback.
- Transmit and receive examples, plus Arduino / PlatformIO packaging.

[1.3.0]: https://github.com/mfurga/cc1101/compare/1.2.2...1.3.0
[1.2.2]: https://github.com/mfurga/cc1101/compare/1.2.1...1.2.2
[1.2.1]: https://github.com/mfurga/cc1101/compare/1.2.0...1.2.1
[1.2.0]: https://github.com/mfurga/cc1101/compare/1.1.5...1.2.0
[1.1.5]: https://github.com/mfurga/cc1101/compare/1.1.4...1.1.5
[1.1.4]: https://github.com/mfurga/cc1101/compare/1.1.3...1.1.4
[1.1.3]: https://github.com/mfurga/cc1101/compare/1.1.2...1.1.3
[1.1.2]: https://github.com/mfurga/cc1101/compare/1.1.1...1.1.2
[1.1.1]: https://github.com/mfurga/cc1101/compare/1.1.0...1.1.1
[1.1.0]: https://github.com/mfurga/cc1101/compare/1.0.5...1.1.0
[1.0.5]: https://github.com/mfurga/cc1101/releases/tag/1.0.5
[1.0.4]: https://github.com/mfurga/cc1101/commit/c246d6f
[1.0.3]: https://github.com/mfurga/cc1101/commit/ec3a599
[1.0.2]: https://github.com/mfurga/cc1101/commit/301bd45
[1.0.1]: https://github.com/mfurga/cc1101/commit/00c9b61
