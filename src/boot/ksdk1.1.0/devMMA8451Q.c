/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.
	2023 Andreas Economides.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devMMA8451Q.h"

extern volatile WarpI2CDeviceState deviceMMA8451QState;
extern volatile uint32_t gWarpI2cBaudRateKbps;
extern volatile uint32_t gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t gWarpSupplySettlingDelayMilliseconds;

#define SIGN_EXTEND(value, bits) ((value ^ (1 << (bits - 1))) - (1 << (bits - 1)))

#define MMA8451Q_FIFO_SIZE 1

#define MMA8451Q_STATUS_REGISTER 0x00
#define MMA8451Q_FIFO_POINTER_REGISTER 0x01

struct __attribute__((packed)) ReadingsRaw
{
	uint16_t x_raw;
	uint16_t y_raw;
	uint16_t z_raw;
};

/* Should have an identical layout to struct ReadingsRaw. */
struct __attribute__((packed)) Readings
{
	int16_t x;
	int16_t y;
	int16_t z;
};

static int16_t process_reading(uint16_t raw)
{
	/*
	 * We read data in big-endian order, but the ARM architecture is
	 * little-endian. Use the REV16 instruction to swap the byte order. This
	 * much more efficient than shifting and oring bits together (the
	 * compiler won't optimize it, I checked).
	 *
	 * See https://developer.arm.com/documentation/dui0489/i/arm-and-thumb-instructions/rev16.
	 */
	const uint16_t swapped = __builtin_bswap16(raw);

	/* The 2 LSBs are always 0s, as readings are 14 bits. */
	const uint16_t shifted = swapped >> 2;

	/* Finally sign-extend the reading. */
	return (int16_t)((shifted ^ (1 << 13)) - (1 << 13));
}

static struct Readings *process_readings(struct ReadingsRaw *const raw)
{
	/*
	 * Use the type-checker to prevent unintentional accesses to raw
	 * readings.
	 */
	ASSERT(raw);

	raw->x_raw = process_reading(raw->x_raw);
	raw->y_raw = process_reading(raw->y_raw);
	raw->z_raw = process_reading(raw->z_raw);

	return (struct Readings *)raw;
}

void initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceMMA8451QState.i2cAddress = i2cAddress;
	deviceMMA8451QState.operatingVoltageMillivolts = operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	// clang-format off
	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
		{
			// clang-format on
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMMA8451Q(void)
{
	WarpStatus i2c_status = 0;

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	/*
	 * 0x09: F_SETUP FIFO setup register
	 *
	 * F_MODE[1:0] = 01     => Circular buffer
	 * F_WMRK[5:0] = 000000 => No watermark
	 */
	// i2c_status |= writeSensorRegisterMMA8451Q(0x09, 0b01000000);

	/*
	 * 0x0E: XYZ_DATA_CFG register
	 *
	 * RESERVED[2:0] = 000
	 * HPF_OUT = 1		=> Output data is high-pass filtered
	 * RESERVED[1:0] = 000
	 * FS[1:0] = 00		=> 2g scale
	 */
	// i2c_status |= writeSensorRegisterMMA8451Q(0x0E, 0b000100000);

	/*
	 * 0x2A: CTRL_REG1 system control 1 register
	 *
	 * ASLP_RATE[1:0] = 00	=> 50 Hz
	 * DR[2:0] = 000	=> 800 Hz data rate
	 * LNOISE = 0		=> Normal mode
	 * F_READ = 0		=> Normal mode
	 * ACTIVE = 1		=> Active mode
	 */
	i2c_status |= writeSensorRegisterMMA8451Q(0x2A, 0b00000001);

	return i2c_status;
}

static WarpStatus
read_register(uint8_t device_register, int number_of_bytes, uint8_t *out)
{
	uint8_t cmd_buf[1] = {0xFF};
	i2c_status_t status;

	uint8_t *const data_buf = out != NULL
				      ? out
				      : (uint8_t *)deviceMMA8451QState.i2cBuffer;

	// clang-format off
	switch (device_register)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
		// clang-format on
		{
			/* OK */
			break;
		}

		default:
		{
		// return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	cmd_buf[0] = device_register;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
	    0 /* I2C peripheral instance */,
	    &slave,
	    cmd_buf,
	    1,
	    data_buf,
	    number_of_bytes,
	    100);
	//     gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		warpPrint("i2c Status: %d", status);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
{
	return read_register(deviceRegister, numberOfBytes, NULL);
}

void printSensorDataMMA8451Q(bool hexModeFlag)
{
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
}

void startLoopMMA8451Q(void)
{
	struct ReadingsRaw buffer[MMA8451Q_FIFO_SIZE];

	while (true)
	{
		WarpStatus i2c_status = 0;

		i2c_status = read_register(MMA8451Q_STATUS_REGISTER,
					   1, NULL);

		if (i2c_status != kWarpStatusOK)
		{
			warpPrint("i2c read failed %d\n", i2c_status);
			return;
		}

		warpPrint("STATUS 0x%02x\n", deviceMMA8451QState.i2cBuffer[0]);

		// i2c_status = read_register(MMA8451Q_FIFO_POINTER_REGISTER,
		// 			   1, NULL);

		// if (i2c_status != kWarpStatusOK)
		// {
		// 	warpPrint("i2c read failed %d\n", i2c_status);
		// 	return;
		// }

		// const uint8_t fifo_pointer = deviceMMA8451QState.i2cBuffer[0];

		// warpPrint("POINTER 0x%02x\n", fifo_pointer);

		// i2c_status = read_register(fifo_pointer,
		i2c_status = read_register(0x01,
					   sizeof buffer, (uint8_t *)buffer);

		if (i2c_status != kWarpStatusOK)
		{
			warpPrint("i2c read failed %d\n", i2c_status);
			return;
		}

		warpPrint("BEGIN\n");

		for (int i = 0; i < MMA8451Q_FIFO_SIZE; ++i)
		{
			const struct Readings *readings = process_readings(&buffer[i]);
			warpPrint("X: %d Y: %d Z: %d\n",
				  readings->x,
				  readings->y,
				  readings->z);
		}

		warpPrint("END\n");

		OSA_TimeDelay(500);
	}
}
