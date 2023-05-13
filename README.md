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

The CC1101 exposes also two general purpose pins (GDO0, GDO2) which can be used to interrupt the MCU on certain events (e.g. RX FIFO is filled). They are optional and not required for proper work.
