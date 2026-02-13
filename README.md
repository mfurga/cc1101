# CC1101

The Arduino library for the TI CC1101 sub-1 Ghz RF transceiver.

CC1101 key features:
* Modulations: ASK (OOK), 2-FSK, GFSK, 4-FSK, MSK
* Frequency bands: 300-348 MHz, 387-464 MHz, 779-928 MHz
* Data rate: 0.6 to 600 kbps
* Output power up to +12 dBm
* Support for sync word detection, address check, flexible packet length, and automatic CRC handling

For more information check out the [datasheet](https://www.ti.com/lit/ds/symlink/cc1101.pdf).

## Hardware connection

The CC1101 module uses the SPI interface for communication. The SI, SO, SCLK pins should be connected to dedicated SPI pins on your Arduino board. The CSn can be connected to any digital pin.

On an ESP32 any GPIOs can be used, you can specify alternate ones in the constructor.

The CC1101 exposes also two general purpose pins (GDO0, GDO2) which can be used to interrupt the MCU on certain events (e.g. RX FIFO is filled). They are optional and not required for proper work.


## Software reference

### Radio configuration

#### setModulation
```cpp
void setModulation(Modulation mod)
```
Sets the modulation. Allowed modulations: `MOD_2FSK`, `MOD_GFSK`, `MOD_ASK_OOK`, `MOD_4FSK`, `MOD_MSK`.

#### setFrequency
```cpp
Status setFrequency(double freq)
```
Sets the frequency (in MHz). Allowed frequency bands: 300-348 MHz, 387-464 MHz, 779-928 MHz.

Returns `STATUS_INVALID_PARAM` on bad frequency.


#### setFrequencyDeviation
```cpp
Status setFrequencyDeviation(double dev)
```
Sets the frequency deviation. Allowed frequency deviation range: 1.587 to 380.859 kHz (assuming 26.0 MHz crystal frequency). For `MOD_ASK_OOK` modulation this setting has no effect.

Returns `STATUS_INVALID_PARAM` on bad frequency deviation.

#### setChannel
```cpp
void setChannel(uint8_t ch)
```
Sets the channel frequency. The channel frequency is multiplied by the channel spacing setting and added to the base frequency.

#### setChannelSpacing
```cpp
Status setChannelSpacing(double sp)
```
Sets the channel spacing frequency (in kHz). Allowed channel spacing range: 25.390 to 405.456 kHz (assuming 26.0 MHz crystal frequency).

Returns `STATUS_INVALID_PARAM` on bad channel spacing.

#### setDataRate
```cpp
Status setDataRate(double drate)
```
Sets the data rate (in kBaud). The allowed data rate depends on the selected modulation (see General Characteristics Table in the datasheet).

Returns `STATUS_INVALID_PARAM` on bad data rate.

#### setRxBandwidth
```cpp
Status setRxBandwidth(double bw);
```
Sets the receiver channel filter bandwidth (in kHz). Allowed bandwidth range: 58 to 812 kHz (assuming 26.0 MHz crystal frequency).

Returns `STATUS_INVALID_PARAM` on bad bandwidth.

#### setOutputPower
```cpp
void setOutputPower(int8_t power)
```
Sets the RF output power (in dBm). Allowed output powers: -30, -20, -15, -10, 0, 5, 7 and 10 dBm.

#### transmit
```cpp
Status transmit(uint8_t *data, size_t length, uint8_t addr = 0)
```
Transmits the data. The `addr` parameter is used when the address filtering mode is enabled.

Returns
* `STATUS_LENGTH_TOO_BIG` if `length` parameter is greater than 255, or if the `length` is greater than required in fixed packet mode
* `STATUS_LENGTH_TOO_SMALL` if the `length` is smaller than required in fixed packet mode
* `STATUS_TXFIFO_UNDERFLOW` if the TX FIFO buffer runs out of data during transmission

#### receive
```cpp
Status receive(uint8_t *data, size_t length, size_t *read = nullptr, uint8_t addr = 0)
```
Receives the data. The `read` parameter indicates the number of bytes actually received. The `addr` parameter is used when the address filtering mode is enabled.

Returns
* `STATUS_LENGTH_TOO_BIG` if the `length` parameter is greater than 255
* `STATUS_LENGTH_TOO_SMALL` if `data` buffer is too small to hold the entire packet
* `STATUS_CRC_MISMATCH` if the received CRC does not match the calculated CRC
* `STATUS_RXFIFO_OVERFLOW` if the RX FIFO overflows due to unread incoming data

#### getRSSI
```cpp
int8_t getRSSI()
```

Returns RSSI (Received Signal Strength Indicator) of the last received packet.

#### getLQI
```cpp
uint8_t getLQI()
```

Returns LQI (Link Quality Indicator) of the last received packet.

### Packet format configuration

![Packet format](https://raw.githubusercontent.com/mfurga/cc1101/main/assets/packet.png)


#### setSyncMode
```cpp
void setSyncMode(SyncMode mode)
```
Sets the sync word mode.

* `SYNC_MODE_NO_PREAMBLE` - Disables preamble and sync word transmission in TX and preamble and sync word detection in RX.
* `SYNC_MODE_15_16` - Enables 16-bit sync word transmission in TX and 16-bits sync word detection in RX. Only 15 of 16 bits need to match in RX.
* `SYNC_MODE_16_16` - Enables 16-bit sync word transmission in TX and 16-bits sync word detection in RX. All 16 bits need to match in RX.
* `SYNC_MODE_30_32` - Enables repeated sync word transmission in TX and 32-bits sync word detection in RX. Only 30 of 32 bits need to match in RX. The sync word will then be repeated twice.
* `SYNC_MODE_NO_PREAMBLE_CS` - Disables preamble and sync word transmission in TX and preamble and sync word detection in RX.
* `SYNC_MODE_15_16_CS` - Enables 16-bit sync word transmission in TX and 16-bits sync word detection in RX. Only 15 of 16 bits need to match in RX. Requires carrier sense above threshold in addition to sync word.
* `SYNC_MODE_16_16_CS` - Enables 16-bit sync word transmission in TX and 16-bits sync word detection in RX. All 16 bits need to match in RX. Requires carrier sense above threshold in addition to sync word.
* `SYNC_MODE_30_32_CS` - Enables repeated sync word transmission in TX and 32-bits sync word detection in RX. Only 30 of 32 bits need to match in RX. The sync word will then be repeated twice. Requires carrier sense above threshold in addition to sync word.

#### setPreambleLength
```cpp
Status setPreambleLength(uint8_t length)
```
Sets the preamble length (in bits). Allowed lengths: 16, 24, 32, 48, 64, 96, 128 and 192.

Returns `STATUS_INVALID_PARAM` on bad length.

#### setSyncWord
```cpp
void setSyncWord(uint16_t sync)
```
Sets the 16-bit sync word.

#### setPacketLengthMode
```cpp
void setPacketLengthMode(PacketLengthMode mode, uint8_t length = 255)
```
Sets the packet length mode. Packet length types:

* `PKT_LEN_MODE_FIXED` - Fixed packet length mode. The length field is not transmitted in TX and `length` parameter indicates the number of bytes that handler will accept in RX.
* `PKT_LEN_MODE_VARIABLE` - Variable packet length mode. The length field is transmitted in TX. The packet handler assumes that the first byte (after the sync word) is the length byte and receives the number of bytes indicated by its value. The `length` parameter is used to set the maximum packet length allowed in RX. Any packet received with a length byte with a value greater than `length` will be discarded.

> [!IMPORTANT]
> The library supports only packets up to 255 bytes.

#### setAddressFilteringMode
```cpp
void setAddressFilteringMode(AddressFilteringMode mode)
```
Sets the address filtering mode.

* `ADDR_FILTER_MODE_NONE` - Disables address transmission in TX and address checking in RX.
* `ADDR_FILTER_MODE_CHECK` - Enables address transmission in TX and address checking in RX. No broadcast address.
* `ADDR_FILTER_MODE_CHECK_BC_0` - Enables address transmission in TX and address checking in RX. Address 0 is broadcast address.
* `ADDR_FILTER_MODE_CHECK_BC_0_255` - Enables address transmission in TX and address checking in RX. Addresses 0 and 255 are broadcast addresses.

#### setCrc
```cpp
void setCrc(bool enable)
```
Enables/disables CRC calculation in TX and CRC checking in RX.

#### setDataWhitening
```cpp
void setDataWhitening(bool enable)
```

Enables/disables data whitening.

If whitening is enabled, everything following the sync words will be whitened. This is done before the optional FEC/Interleaver stage.

#### setManchester
```cpp
Status setManchester(bool enable)
```

Enables/disables Manchester encoding in TX and Manchester decoding in RX.

> [!NOTE]
> Manchester encoding is not supported at the same time as using the FEC/Interleaver option or when using MSK
and 4-FSK modulation.

Returns `STATUS_BAD_STATE` if Manchester encoding cannot be enabled.

#### setFEC
```cpp
Status setFEC(bool enable)
```

Enables/disables Forward Error Correction (FEC) with interleaving for the packet payload.

> [!NOTE]
> Only supported for fixed packet length mode and when Manchester encoding is disabled.

Returns `STATUS_BAD_STATE` if FEC cannot be enabled.

### Direct register access

> [!WARNING]
> The following methods are not part of the standard library API. Modifying register values directly may cause other library methods to work incorrectly. Use these methods only when the desired functionality is not available through the standard API. Refer to the [datasheet](https://www.ti.com/lit/ds/symlink/cc1101.pdf) and the library's source code before use to ensure the changes do not conflict with the library's internal state.

#### readReg
```cpp
uint8_t readReg(uint8_t addr)
```
Reads a single register value.

#### readRegField
```cpp
uint8_t readRegField(uint8_t addr, uint8_t hi, uint8_t lo)
```
Reads a specific bit field from a register. The `hi` and `lo` parameters specify the inclusive bit range.

#### readRegBurst
```cpp
void readRegBurst(uint8_t addr, uint8_t *buff, size_t size)
```
Reads multiple consecutive registers into a buffer.

#### writeReg
```cpp
void writeReg(uint8_t addr, uint8_t data)
```
Writes a single register value.

#### writeRegField
```cpp
void writeRegField(uint8_t addr, uint8_t data, uint8_t hi, uint8_t lo)
```
Writes a value to a specific bit field of a register. The `hi` and `lo` parameters specify the inclusive bit range.

#### writeRegBurst
```cpp
void writeRegBurst(uint8_t addr, uint8_t *data, size_t size)
```
Writes multiple consecutive registers from a buffer.

##

The library was tested on the following boards:

 * [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)
 * [Arduino Pro Mini 3.3V](https://docs.arduino.cc/retired/boards/arduino-pro-mini/)
 * [Wemos D1 Mini ESP32](https://docs.platformio.org/en/stable/boards/espressif32/wemos_d1_mini32.html)
 * [ESP32C3 Super Mini](https://www.espboards.dev/esp32/esp32-c3-super-mini/)

