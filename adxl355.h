/*
 * Copyright (c) Wolfgang Neue
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ADXL355_H
#define ADXL355_H

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Internal flags for adxl355_is_... / adxl355_has_... (status-related)
#define _ADXL355_STATUS_NVM_BUSY	1<<4
#define _ADXL355_STATUS_ACTIVITY	1<<3
#define _ADXL355_STATUS_FIFO_OVR	1<<2
#define _ADXL355_STATUS_FIFO_FULL	1<<1
#define _ADXL355_STATUS_DATA_RDY	0

// Indices for adxl355_axis_get_data_raw() and adxl355_axis_get_data_raw_timeout()
#define ADXL355_INDEX_X				0
#define ADXL355_INDEX_Y				1
#define ADXL355_INDEX_Z				2

// Flags for adxl355_general_settings() / adxl355_is_... (settings-related)
#define ADXL355_I2C_HIGH_SPEED		1<<7
#define ADXL355_I2C_FAST			0
#define ADXL355_INT_ACTIVE_HIGH		1<<6
#define ADXL355_INT_ACTIVE_LOW		0
#define ADXL355_RANGE_2G			1
#define ADXL355_RANGE_4G			2
#define ADXL355_RANGE_8G			3

/* ------------------------------------------------------------------------------------- */
/* - Data types - */

/*
 * Represents a single adxl345 sensor on the I²C bus
 */
typedef struct
{
	i2c_inst_t *i2c_port;
	uint8_t i2c_addr;
} adxl355_sensor;

/*
 * Represents a single set of values read from a sensor
 * (x-, y- and z-axis in this order)
 */
typedef union
{
	int32_t axis[3];
	uint8_t raw[12];
} adxl355_axis_data;

/* ------------------------------------------------------------------------------------- */

// Always 0xAD
uint8_t adxl355_get_company_id (adxl355_sensor *sensor);
// Always 0x1D
uint8_t adxl355_get_mems_id (adxl355_sensor *sensor);
// Always 0xED (355 in octal, because it's the adxl355)
uint8_t adxl355_get_device_id (adxl355_sensor *sensor);

uint8_t adxl355_get_revision_id (adxl355_sensor *sensor);

uint8_t adxl355_get_status (adxl355_sensor *sensor);
#define adxl355_is_nvm_busy(status)				((status & _ADXL355_STATUS_NVM_BUSY) != 0)
#define adxl355_is_activity_detected(status)	((status & _ADXL355_STATUS_ACTIVITY) != 0)
#define adxl355_has_fifo_overrun(status)		((status & _ADXL355_STATUS_FIFO_OVR) != 0)
#define adxl355_is_watermark_reached(status)	((status & _ADXL355_STATUS_FIFO_FULL) != 0)
#define adxl355_is_data_ready(status)			((status & _ADXL355_STATUS_DATA_RDY) != 0)

uint8_t adxl355_get_num_fifo_entries (adxl355_sensor *sensor);

uint16_t adxl355_get_temp (adxl355_sensor *sensor);
bool adxl355_get_temp_raw (adxl355_sensor *sensor, uint8_t *temp);

int adxl355_axis_get_data (adxl355_sensor *sensor, adxl355_axis_data *data);
int adxl355_axis_get_data_timeout (adxl355_sensor *sensor, adxl355_axis_data *data, unsigned int timeout_s);
int adxl355_axis_get_data_raw (adxl355_sensor *sensor, uint8_t axis, uint8_t *data);
int adxl355_axis_get_data_raw_timeout (adxl355_sensor *sensor, uint8_t axis, uint8_t *data, unsigned int timeout_s);

int adxl355_get_x (adxl355_axis_data *data);
int adxl355_get_y (adxl355_axis_data *data);
int adxl355_get_z (adxl355_axis_data *data);

bool adxl355_general_settings (adxl355_sensor *sensor, uint8_t flags);
bool adxl355_power_settings (adxl355_sensor *sensor, uint8_t flags);

uint8_t adxl355_get_general_settings (adxl355_sensor *sensor);
#define adxl355_is_i2c_high_speed (settings)		((settings & ADXL355_I2C_HIGH_SPEED) != 0)
#define adxl355_is_int_active_high (settings)	((settings & ADXL355_INT_ACTIVE_HIGH) != 0)
#define adxl355_is_range_at_2g (settings)		((settings & ADXL355_RANGE_2G) != 0)
#define adxl355_is_range_at_4g (settings)		((settings & ADXL355_RANGE_4G) != 0)
#define adxl355_is_range_at_8g (settings)		((settings & ADXL355_RANGE_8G) != 0)

/* ------------------------------------------------------------------------------------- */
/* - Advanced darkness - */
// It's easier to use the functions above, but feel free

#define ADXL355_REG_DEVID_AD		0x00		// Read
#define ADXL355_REG_DEVID_MST		0x01		// Read
#define ADXL355_REG_PARTID			0x02		// Read
#define ADXL355_REG_REVID			0x03		// Read
#define ADXL355_REG_STATUS			0x04		// Read
#define ADXL355_REG_FIFO_ENTRIES	0x05		// Read
#define ADXL355_REG_TEMP2			0x06		// Read
#define ADXL355_REG_TEMP1			0x07		// Read
#define ADXL355_REG_XDATA3			0x08		// Read
#define ADXL355_REG_XDATA2			0x09		// Read
#define ADXL355_REG_XDATA1			0x0A		// Read
#define ADXL355_REG_YDATA3			0x0B		// Read
#define ADXL355_REG_YDATA2			0x0C		// Read
#define ADXL355_REG_YDATA1			0x0D		// Read
#define ADXL355_REG_ZDATA3			0x0E		// Read
#define ADXL355_REG_ZDATA2			0x0F		// Read
#define ADXL355_REG_ZDATA1			0x10		// Read
#define ADXL355_REG_FIFO_DATA		0x11		// Read
// 0x12 - 0x1D seems to be not in use
#define ADXL355_REG_OFFSET_X_H		0x1E		// Read / Write
#define ADXL355_REG_OFFSET_X_L		0x1F		// Read / Write
#define ADXL355_REG_OFFSET_Y_H		0x20		// Read / Write
#define ADXL355_REG_OFFSET_Y_L		0x21		// Read / Write
#define ADXL355_REG_OFFSET_Z_H		0x22		// Read / Write
#define ADXL355_REG_OFFSET_Z_L		0x23		// Read / Write
#define ADXL355_REG_ACT_EN			0x24		// Read / Write
#define ADXL355_REG_ACT_THRESH_H	0x25		// Read / Write
#define ADXL355_REG_ACT_THRESH_L	0x26		// Read / Write
#define ADXL355_REG_ACT_COUNT		0x27		// Read / Write
#define ADXL355_REG_FILTER			0x28		// Read / Write
#define ADXL355_REG_FIFO_SAMPLES	0x29		// Read / Write
#define ADXL355_REG_INT_MAP			0x2A		// Read / Write
#define ADXL355_REG_SYNC			0x2B		// Read / Write
#define ADXL355_REG_RANGE			0x2C		// Read / Write
#define ADXL355_REG_POWER_CTL		0x2D		// Read / Write
#define ADXL355_REG_SELF_TEST		0x2E		// Read / Write
#define ADXL355_REG_RESET			0x2F		// Write

#define ADXL355_REG_DATA_BEGIN_AXIS	ADXL355_REG_XDATA3

#define ADXL355_REG_DATA_BEGIN_X	ADXL355_REG_XDATA3
#define ADXL355_REG_DATA_BEGIN_Y	ADXL355_REG_YDATA3
#define ADXL355_REG_DATA_BEGIN_Z	ADXL355_REG_ZDATA3

int adxl355_write (adxl355_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop);

int adxl355_write_timeout (adxl355_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop, unsigned int timeout_s);

int adxl355_read (adxl355_sensor *sensor, uint8_t reg_addr, uint8_t *buffer, size_t length, bool no_stop);

int adxl355_read_timeout (adxl355_sensor *sensor, uint8_t reg_addr, uint8_t *buffer, size_t length, bool no_stop, unsigned int timeout_s);

#endif