/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "adxl382.h"
#include <errno.h>

/* Example code to talk to a ADXL382 acceleromater sensor via SPI.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore SPI) cannot be used at 5v.

   You will need to use a level shifter on the SPI lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board and a generic adxl382 board, other
   boards may vary.

   GPIO 16 (pin 21) MISO/spi0_rx-> SDO/SDO on adxl382 board
   GPIO 17 (pin 22) Chip select -> CSB/!CS on adxl382 board
   GPIO 18 (pin 24) SCK/spi0_sclk -> SCL/SCK on adxl382 board
   GPIO 19 (pin 25) M OSI/spi0_tx -> SDA/SDI on adxl382 board
   3.3v (pin 36) -> VS 3V3 pin on adxl382 board
   GND (pin 38)  -> GND on adxl382 board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.

*/

#define DEBUG_PRINT(msg, ...)         \
    do {                              \
        printf(msg, ##__VA_ARGS__);   \
        fflush(stdout);               \
    } while (0)

#define STREAM_PRINT(...)       \
    do {                        \
        printf(__VA_ARGS__);    \
        fflush(stdout);         \
    } while(0)

int ret;
uint8_t register_value;
uint8_t status0;
uint8_t fifo_status[2];
uint8_t fifo_data[954];
uint16_t set_fifo_entries = 0x0C; //  12 entries in FIFO
// uint16_t set_fifo_entries = 0x5A; //  90 entries in FIFO
uint16_t fifo_entries = 2;
bool chID_enable = true; //FIFO channel id
uint8_t fifo_read_bytes;
struct adxl38x_fractional_val data_frac[15];
static char getaxis(uint8_t chID);
uint32_t count = 0;
char buffer[64];
int pos = 0;


int main() {
    // stdio_init_all();
    // sleep_ms(2000);
    // DEBUG_PRINT("Hello from Pico!\n");

    // while (true) {
    //     DEBUG_PRINT("Heartbeat...\n");
    //     sleep_ms(1000);
    // }
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    stdio_init_all();
    sleep_ms(1000);
    
    if (!stdio_usb_connected()) {
        DEBUG_PRINT("USB not connected yet\n");
    }
    else
        DEBUG_PRINT("USB port is successfully initialised\n");
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/adxl382_spi example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else

    DEBUG_PRINT("Hello, adxl382! Reading raw data from registers via SPI...\n");

    // This example will use SPI0 at 8MHz.
    spi_init(spi_default, 8000 * 1000);
    // Set SPI format
    spi_set_format( spi0,   // SPI instance
                    8,      // Number of bits per transfer
                    0,      // Polarity (CPOL)
                    0,      // Phase (CPHA)
                    SPI_MSB_FIRST);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    if (spi_is_writable(spi_default)) {
    DEBUG_PRINT("SPI is writable\n");
    }

	
    sleep_ms(1000);
    ret = adxl38x_init();
    
	if (ret) {
		DEBUG_PRINT("Error: couldnt initialize ADXL382\n");
        gpio_put(LED_PIN, 1);
        goto error;
	} else {
		DEBUG_PRINT("ADXL is successfully initialised\n");
	}

        
    
	while (true) {
        if (stdio_usb_connected()) {
            int c = getchar_timeout_us(1000);
            if (c != PICO_ERROR_TIMEOUT) {
                if (c == '\n' || c == '\r') {
                    buffer[pos] = '\0';  // terminate string
                    if (strcmp(buffer, "START") == 0) {
						gpio_put(LED_PIN, 0);
						ret = adxl38x_soft_reset();
						if (ret == -EAGAIN) {
							DEBUG_PRINT("Error: Reset was not successful\n");
							goto error;
						}
						else if (ret)
							goto error;
						ret = adxl38x_set_range(ADXL38X_OP_MODE,ADXL38X_MASK_RANGE, ADXL382_RANGE_15G);
						if (ret)
							goto error;
						else
							DEBUG_PRINT("ADXL382 range is set to 15g\n");
						// FIFO sequence
						// Put the part in standby mode, All configuration register writes must be completed with the ADXL382 in standby mode
						ret = adxl38x_set_to_standby();
						if (ret)
							goto error;
						

						// Set DIG_EN register to 0x78 (Enable XYZ axes and FIFO enable)
						DEBUG_PRINT("Enable XYZ axes and FIFO\n");
						register_value = 0x78;
						ret = write_register(ADXL38X_DIG_EN, 
										&register_value, 1);
						ret = read_register(ADXL38X_DIG_EN, 1, &register_value);
						DEBUG_PRINT("DIG_EN register value: 0x%02X\n", register_value);
						if (ret)
							goto error;

						// Set FIFO_CFG0 to 0x60 (Channel ID enable and FIFO stream mode)
						DEBUG_PRINT("Set FIFO_CFG0 to 0x60 (Channel ID enable and FIFO stream mode, set FIFO_SAMPLES to %d)\n", set_fifo_entries);
						ret = adxl38x_accel_set_FIFO(set_fifo_entries,
										false, ADXL38X_FIFO_STREAM, chID_enable, false);

						// Set INT0_MAP0 to 0x08 (FIFO_WATERMARK_INT0)
						DEBUG_PRINT("Set INT0_MAP0 to 0x08 (FIFO_WATERMARK_INT0)\n");
						register_value = 0x08;
						ret = write_register( ADXL38X_INT0_MAP0, &register_value, 1);
						if (ret) {
							DEBUG_PRINT("Error: Error in setting INT0_MAP0\n");
							goto error;
						}

                        // Put the part in HP mode and read data when FIFO watermark pin is set
						DEBUG_PRINT("Set HP mode\n");
						ret = adxl38x_set_op_mode(ADXL38X_OP_MODE, ADXL38X_MASK_OP_MODE, ADXL38X_MODE_HP);
						if (ret) {
							DEBUG_PRINT("Error: Error in setting HP_MODE\n");
							goto error;
						}
						else
						DEBUG_PRINT("Device is in HP mode\n");
						
						DEBUG_PRINT("Starting watermark check\n");
						//reset count
						count = 0;
						while (count < 96000) { //loop until 32k x,y,z samples are obtained (2seconds at 16kHz)
							
							// Read status to assert if FIFO_WATERMARK bit set
							ret = read_register(ADXL38X_STATUS0, 1, &status0);
							if (ret)
								goto error;
							// DEBUG_PRINT("Status 0: %x\n", status0);
							ret = read_register(ADXL38X_FIFO_STATUS0, 2, fifo_status);
							if (ret)
								goto error;
							fifo_entries = (fifo_status[0] | ((uint16_t)fifo_status[1] << 8));
							fifo_entries = fifo_entries & 0x01ff;

							//clear fifo_data buffer
							memset(fifo_data, 0, sizeof(fifo_data));

							// Read FIFO status and data if FIFO_WATERMARK is set
							if (status0 & (1<<3)) {
								count += set_fifo_entries;
								// DEBUG_PRINT("FIFO_WATERMARK bit is set\n");
								//DEBUG_PRINT("Fifo entries =  %d\n", fifo_entries);
								if (fifo_entries < set_fifo_entries)
									goto unmatch_error;

								// Read data from FIFO (can read at least upto 12 samples * 3 bytes (chID, data))
								if (chID_enable)
									fifo_read_bytes = 3;
								// DEBUG_PRINT("Reading %d bytes from FIFO\n", set_fifo_entries*fifo_read_bytes);
								ret = read_register(ADXL38X_FIFO_DATA, set_fifo_entries*fifo_read_bytes, fifo_data);
								if (ret)
									goto error;
								// DEBUG_PRINT("FIFO data read successfully\n");
							
								// DEBUG_PRINT("Sample Count = %d\n", count);
							
								// DEBUG_PRINT("FIFO data:\n");
								// print raw fifo data
								for (int b = 0; b < set_fifo_entries*fifo_read_bytes; b += fifo_read_bytes) {
									DEBUG_PRINT("%02X%02X%02X\n", fifo_data[b], fifo_data[b+1], fifo_data[b+2]);
								}
								// for (int b = 0; b < set_fifo_entries*fifo_read_bytes; b += 3) {
								// 	ret = adxl38x_data_raw_to_gees((fifo_data + b + 1), data_frac, ADXL382_RANGE_15G);
								// 	if (ret)
								// 		goto error;
								// 	STREAM_PRINT("%c : %lld.%04d\n", getaxis(fifo_data[b]), data_frac->integer,
								// 		labs(data_frac->fractional));
								// }
							}
							
						}
						DEBUG_PRINT("End\n");
						gpio_put(LED_PIN, 1);
						//put device in standby mode
						ret = adxl38x_set_to_standby();
						if (ret)
							goto error;
                    }
                    pos = 0;  // reset buffer
                } else if (pos < 64 - 1) {
                    buffer[pos++] = c;
                }
            }
        }
    }
	
error:
	if (ret)
		DEBUG_PRINT("Error: Error occurred!\n");
	else
		DEBUG_PRINT("The program has ended after successful execution\n");
	return 0;
unmatch_error:
	DEBUG_PRINT("Error: Number of entries in FIFO not matching the number set in FIFO config\n");
	return 0;


#endif
}

/***************************************************************************//**
 * @brief Assigns axis based on channel index
 *
 * @param chID         - Channel index
 *
 * @return ret         - Corresponding channel ID for channel index provided
*******************************************************************************/
static char getaxis(uint8_t chID)
{
	if (chID)
		return chID > 1 ? 'z' : 'y';
	return 'x';
}
