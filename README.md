
This is the C driver code for ADXL382 sensor. The MCU used is raspberry pi pico. Data communicatin is done via SPI.

In the main.c, ADXL382 is set up to operate in High performance mode, 8kHz bandwidth and sampling rate (output data rate) at 16KHz.

# Connect Pico to ADXL382
GPIO 16 (pin 21) MISO/spi0_rx-> SDO/SDO on adxl382 board  <br>
GPIO 17 (pin 22) Chip select -> CSB/!CS on adxl382 board <br>
GPIO 18 (pin 24) SCK/spi0_sclk -> SCL/SCK on adxl382 board <br>
GPIO 19 (pin 25) MOSI/spi0_tx -> SDA/SDI on adxl382 board <br>
3.3v (pin 36) -> 3V3 pin on adxl382 board <br>
GND (pin 38)  -> GND on adxl382 board <br>

# Deploy the binary file main.uf2 to Pico
1. Open VScode
2. Install raspberry pi pico extension
3. Clone this repo to your pc
4. In VScode raspberry pico windows, import this github project
5. Plug the micro-usb cable to Pico while pressing the BOOTSEL button, this sets the pico in bootsel mode
6. Click run button at the bottom right corner of vs code to flash the main.uf2 file to pico
7. Unplug and plug the cable to pico again
8. Open Tera Term and set up COM port
9. See the data printed on teraterm
