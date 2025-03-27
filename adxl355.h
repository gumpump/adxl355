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

typedef struct
{
	i2c_inst_t *i2c_port;
	uint8_t i2c_addr;
} adxl355_sensor;

uint8_t adxl355_get_status (adxl355_sensor *sensor);
#define adxl355_is_nvm_busy(status)				((status & _ADXL355_STATUS_NVM_BUSY) != 0)
#define adxl355_is_activity_detected(status)	((status & _ADXL355_STATUS_ACTIVITY) != 0)
#define adxl355_has_fifo_overrun(status)		((status & _ADXL355_STATUS_FIFO_OVR) != 0)
#define adxl355_is_watermark_reached(status)	((status & _ADXL355_STATUS_FIFO_FULL) != 0)
#define adxl355_is_data_ready(status)			((status & _ADXL355_STATUS_DATA_RDY) != 0)

uint8_t adxl355_get_num_fifo_entries (adxl355_sensor *sensor);
// Function for getting temperature out of TEMP1 and TEMP2

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

#endif