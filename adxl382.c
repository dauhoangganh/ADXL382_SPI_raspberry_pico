/***************************************************************************//**
 *   @file   adxl38x.c
 *   @brief  Implementation of ADXL38X Driver.
 *   @author Anh Do (anh.do@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include "adxl382.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <stdint.h>

static const uint8_t adxl38x_scale_mul[3] = {1, 2, 4};

#ifdef PICO_DEFAULT_SPI_CSN_PIN
static inline void cs_select(void) {
	// __asm__ volatile("nop \n nop \n nop");
	gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);  // Active low
	// __asm__ volatile("nop \n nop \n nop");
}

static inline void cs_deselect(void) {
	// asm volatile("nop \n nop \n nop");
	gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
	// asm volatile("nop \n nop \n nop");
}
#endif

#if defined(spi_default) && defined(PICO_DEFAULT_SPI_CSN_PIN)
int write_register(uint8_t reg, const uint8_t *data, uint8_t len) {
	uint8_t buf[len + 1];
	buf[0] = (reg << 1) | 0x00;  // Remove read bit as this is a write, only the last 7 bits of register address are used, the register address is followed by R/W bit
	for (uint8_t i = 0; i < len; i++) {
		buf[i + 1] = data[i];
	}
	gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
	// sleep_us(1);
	int ret = spi_write_blocking(spi_default, buf, len + 1);
	gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
	// sleep_us(1);
	return ret == len + 1 ? 0 : -1;  // Return 0 on success, -1 on failure
}
int read_register(uint8_t reg, uint8_t len, uint8_t *buf ) {
	// For this particular device, we send the device the register we want to read
	// first, then subsequently read from the device. The register is auto incrementing
	// so we don't need to keep sending the register we want, just the first.
	reg = (reg <<1) | 0x01;  // set read bit
	gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
	// printf("CS low\n");
	// sleep_us(1);
	int ret = spi_write_blocking(spi_default, &reg, 1);
	if (ret != 1) {
		printf("Error: Could not write register address\n");
		gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
		sleep_us(1);
		return -1;  // Return -1 on failure
	}
	// sleep_ms(10);
	ret = spi_read_blocking(spi_default, 0, buf, len);
	gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
	// printf("CS high\n");
	// sleep_us(1);
	return ret == len ? 0 : -1;  // Return 0 on success, -1 on failure
}


#endif


/***************************************************************************//**
 * @brief Initializes the device and checks for valid peripheral communication

 * @return ret         	- Result of the initialization
*******************************************************************************/

int adxl38x_init()
{
	int ret;
	uint8_t reg_value;

	
	ret = read_register(ADXL38X_DEVID_AD,
					1, &reg_value);
	printf("Device ID: %X\n", reg_value);
	// sleep_ms(100);  // Wait for 1ms before retrying
	
	if (ret || (reg_value != ADXL38X_RESET_DEVID_AD)) {
		printf("Error: Couldnt read device ID or Device ID does not match 0xAD\n");
		return -1;
	}
	ret = read_register(ADXL38X_DEVID_MST,
				       1, &reg_value);
	printf("MEMS ID: %X\n", reg_value);
	if (ret || (reg_value != ADXL38X_RESET_DEVID_MST)) {
		printf("Error: Couldnt read DEVID_MST or MEMS devices ID does not match 0x1D\n");
		return -1;
	}
	ret = read_register(ADXL38X_PART_ID,
				       1, &reg_value);
	printf("Part ID: %X\n", reg_value);
	if (ret || (reg_value != ADXL38X_RESET_PART_ID)) {
		printf("Error: Couldnt read partID\n");
		return -1;
	}
	return 0;


}

/***************************************************************************//**
 * @brief Performs a soft reset of the device.
 *
 * @return ret - Result of the soft reset procedure.
*******************************************************************************/
int adxl38x_soft_reset()
{
	uint8_t reg_value;
	int ret;
	uint8_t data = ADXL38X_RESET_CODE;

	// Perform soft reset
	printf("Performing soft reset...\n");
	ret = write_register(ADXL38X_REG_RESET, &data, 1);
	if (ret)
		return ret;
	// Delay is needed between soft reset and initialization (From SOFT_RESET
	// bit description in REG_RESET)
	sleep_us(500);
	ret = read_register(ADXL38X_DEVID_AD, 1, &reg_value);
	printf("Device ID: %X\n", reg_value);
	if (reg_value != 0xAD) {
		printf("Error: Device ID does not match 0xAD\n");
		return -EAGAIN;
	}

	return 0;
}

/***************************************************************************//**
 * @brief Sets the measurement range register value.
 *
 * @param dev       - The device structure.
 * @param range_val - Selected range.
 *
 * @return ret      - Result of the writing procedure.
*******************************************************************************/
int adxl38x_set_range( uint8_t reg_addr, uint8_t mask, enum adxl38x_range range_val)
{
	int ret, update_val;
	uint8_t data;

    ret = read_register(reg_addr, 1, &data);
	if (ret)
		return ret;
	
	// Handle case where the OP_MODE register is accessed. Needs to go to
	// standby before changing any bits of the register. Needed for a safe
	// transition.
	if (reg_addr == ADXL38X_OP_MODE)
		adxl38x_set_to_standby();
    update_val = range_val << 6;
    data &= ~mask;
	data |= update_val;
	return write_register(reg_addr, &data, 1);
}

/***************************************************************************//**
 * @brief Puts the part in a safe state before setting important register values.

 * @return ret      - Result of setting to stanby mode.
*******************************************************************************/
int adxl38x_set_to_standby()
{
	int ret;
	uint8_t op_mode_reg_val;

	ret = read_register(ADXL38X_OP_MODE, 1, &op_mode_reg_val);
	// Applying mask sets underlying op_mode to standby (0)
	op_mode_reg_val = (op_mode_reg_val & ~ADXL38X_MASK_OP_MODE);
	ret = write_register(ADXL38X_OP_MODE,
					 &op_mode_reg_val, 1);
	if (ret)
		printf("Error: Cannot set ADXL382 to standby mode\n");
	else
		printf("ADXL382 is in standby mode\n");
	return ret;
}

/***************************************************************************//**
 * @brief Function to set the paramenters for FIFO mode
 *
 * @param num_samples 		- Number of FIFO entries that FIFI_WATERMARK should set.
 * @param external_trigger 	- Enable/disable external trigger in FIFO stream mode.
 * @param fifo_mode        	- FIFO mode setting.
 * @param ch_ID_enable      - Enable/disable channel ID.
 * @param read_reset       	- reset read/write point and read state machine.
 *
 * @return ret      		- Result of the procedure.
*******************************************************************************/
int adxl38x_accel_set_FIFO(uint16_t num_samples,
			   bool external_trigger, enum adxl38x_fifo_mode fifo_mode, bool ch_ID_enable,
			   bool read_reset)
{
	int ret;
	uint8_t write_data = 0;
	uint8_t fifo_samples_low;
	uint8_t fifo_samples_high;
	uint8_t set_channels;

	// Obtain the channels enabled in DIG_EN register
	ret = read_register( ADXL38X_DIG_EN, 1, &set_channels);
	if (ret)
		return ret;
	set_channels = (set_channels & ADXL38X_MASK_CHEN_DIG_EN)>>4;

	// Check if number of samples provided is allowed
    if (num_samples > 320) {
        printf("Error: Cannot set more than 320 samples in FIFO\n");
        return -EINVAL;
    } else if ((num_samples > 318) &&
         ((!set_channels) || (set_channels == ADXL38X_CH_EN_XYZ) ||
          (set_channels == ADXL38X_CH_EN_YZT))) {
        printf("Error: Cannot set more than 318 samples in FIFO, with current MODEL_CHANNEL_EN setting\n");
        return -EINVAL;
    }

	// set FIFO_CFG1 register
	fifo_samples_low = (uint8_t) num_samples & 0xFF;
	ret = write_register(ADXL38X_FIFO_CFG1, &fifo_samples_low, 1);
	if (ret)
	{
		printf("Error: Cannot set FIFO_CFG1 register\n");
		return ret;
	}
	else
		printf("FIFO_CFG1 register set to %d\n", fifo_samples_low);
	

	// building data for FIFO_CFG0 register
	fifo_samples_high = (uint8_t) (num_samples >> 8);
	fifo_samples_high = fifo_samples_high & 0x01;
	write_data = fifo_samples_high;

	fifo_mode = (fifo_mode << 4) & ADXL38X_FIFOCFG_FIFOMODE_MSK;
	write_data |= fifo_mode;

	if (read_reset)
		write_data |= (1<<7);

	if (ch_ID_enable)
		write_data |= (1<<6);

	if (external_trigger && fifo_mode == ADXL38X_FIFO_TRIGGER)
		write_data |= (1<<3);

	ret = write_register( ADXL38X_FIFO_CFG0, &write_data, 1);
	if (ret)
	{
		printf("Error: Cannot set FIFO_CFG0 register\n");
		return ret;
	}
	else
		printf("FIFO_CFG0 register set to %d\n", write_data);

	return ret;
}

/***************************************************************************//**
 * @brief Places the device into the given operation mode.
 *
 * @param reg_addr - Register address OP_mode.
 * @param op_mode - Operation mode mode.
 *
 * @return ret    - Result of the setting operation procedure.
*******************************************************************************/
int adxl38x_set_op_mode( uint8_t reg_addr, uint8_t mask, enum adxl38x_op_mode op_mode)
{
	int ret, update_val;
	uint8_t data;

    ret = read_register(reg_addr, 1, &data);
	if (ret)
		return ret;
	
	// Handle case where the OP_MODE register is accessed. Needs to go to
	// standby before changing any bits of the register. Needed for a safe
	// transition.
	if (reg_addr == ADXL38X_OP_MODE)
		adxl38x_set_to_standby();
    update_val = op_mode;
    data &= ~mask;
	data |= update_val;
	return write_register(reg_addr, &data, 1);
}

/***************************************************************************//**
 * @brief Function to convert accel data to gees
 *
 * @param raw_accel_data 	- Raw data array of two bytes
 * @param data_frac        	- Fractional data in gees
 *
 * @return ret      		- Result of the procedure.
*******************************************************************************/
int adxl38x_data_raw_to_gees(uint8_t *raw_accel_data,
			     struct adxl38x_fractional_val *data_frac, enum adxl38x_range range_val)
{
	int ret;
	uint16_t data = 0;

	data = raw_accel_data[1] | ((uint16_t)raw_accel_data[0] << 8);
	data_frac->integer = no_os_div_s64_rem((int64_t)adxl38x_accel_conv(data, range_val),
					       ADXL38X_ACC_SCALE_FACTOR_GEE_DIV, &(data_frac->fractional));

	return 0;
}

/***************************************************************************//**
 * @brief Converts raw acceleration value to m/s^2 value.
 *
 * @param dev       - The device structure.
 * @param raw_accel - Raw acceleration value.
 *
 * @return ret      - Converted data.
*******************************************************************************/
int64_t adxl38x_accel_conv(uint16_t raw_accel, enum adxl38x_range range_val)
{
	int32_t accel_data;

	// Raw acceleration is in two's complement
	// Convert from two's complement to int
	if (raw_accel & (1<<15))
		accel_data = (int16_t) raw_accel;
	else
		accel_data = raw_accel;

	// Apply scale factor based on the selected range
	
	return ((int64_t)(accel_data * ADXL382_ACC_SCALE_FACTOR_GEE_MUL *
				  adxl38x_scale_mul[range_val]));
	
}

/**
 * Signed 64bit divide with 32bit divisor with remainder
 */
int64_t no_os_div_s64_rem(int64_t dividend, int32_t divisor, int32_t *remainder)
{
	*remainder = dividend % divisor;
	return dividend / divisor;
}
