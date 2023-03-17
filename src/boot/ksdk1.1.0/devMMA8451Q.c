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

/*
 * Uses a GCC extension to return result from the expression. This simplifies
 * error handling. See https://gcc.gnu.org/onlinedocs/gcc/Statement-Exprs.html.
 */
#define MUST(expr) ({                                                                 \
	const WarpStatus result = expr;                                               \
                                                                                      \
	if (result != kWarpStatusOK)                                                  \
	{                                                                             \
		warpPrint("Evaluation of %s failed (rc = %d)\n", #expr, (int)result); \
		return;                                                               \
	}                                                                             \
                                                                                      \
	result;                                                                       \
})

/*
 * Like MUST(), but propagates errors.
 */
#define TRY(expr) ({                    \
	const WarpStatus result = expr; \
                                        \
	if (result != kWarpStatusOK)    \
		return result;          \
                                        \
	result;                         \
})

#define MMA8451Q_FIFO_SIZE 32

#define MMA8451Q_STATUS_REGISTER 0x00
#define MMA8451Q_FIFO_POINTER_REGISTER 0x01
#define MMA8451Q_REGISTER_MAX 0x31

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
	uint8_t payloadByte[1], commandByte[1];
	i2c_status_t status;

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
		.baudRate_kbps = gWarpI2cBaudRateKbps};

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
	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	/* Need to enter Standby mode first. */
	TRY(writeSensorRegisterMMA8451Q(0x2A, 0b00100000));

	/*
	 * 0x09: F_SETUP FIFO setup register
	 *
	 * F_MODE[1:0] = 01     => Circular buffer
	 * F_WMRK[5:0] = 000000 => Watermark disabled
	 */
	TRY(writeSensorRegisterMMA8451Q(0x09, 0b01000000));

	/*
	 * 0x0E: XYZ_DATA_CFG register
	 *
	 * RESERVED[2:0] = 000
	 * HPF_OUT = 1		=> Output data is high-pass filtered
	 * RESERVED[1:0] = 00
	 * FS[1:0] = 00		=> 2g scale
	 */
	TRY(writeSensorRegisterMMA8451Q(0x0E, 0b00010000));

	/*
	 * 0x0F: HP_FILTER_CUTOFF high-pass filter register
	 *
	 * RESERVED[1:0]
	 * Pulse_PHF_BYP = 0	=> HPF enabled for pulse processing
	 * Pulse_LPF_EN = 0	=> LPF disabled for pulse processing
	 * RESERVED[1:0] = 00
	 * SEL[1:0] = 11	=> 0.25 Hz HPF cut-off frequency (@ 50 Hz)
	 */
	TRY(writeSensorRegisterMMA8451Q(0x0F, 0b00000011));

	/*
	 * 0x2A: CTRL_REG1 system control 1 register
	 *
	 * ASLP_RATE[1:0] = 00	=> 50 Hz
	 * DR[2:0] = 000	=> 50 Hz data rate
	 * LNOISE = 0		=> Normal mode
	 * F_READ = 0		=> Normal mode
	 * ACTIVE = 1		=> Active mode
	 */
	TRY(writeSensorRegisterMMA8451Q(0x2A, 0b00100001));

	warpPrint("Configured!\n");

	return kWarpStatusOK;
}

static WarpStatus
read_register(uint8_t device_register, int number_of_bytes, uint8_t *out)
{
	uint8_t cmd_buf[1] = {0xFF};
	i2c_status_t status;

	uint8_t *const data_buf = out != NULL
				      ? out
				      : (uint8_t *)deviceMMA8451QState.i2cBuffer;

	/* All valid registers can be read. */
	if (device_register > MMA8451Q_REGISTER_MAX)
		return kWarpStatusBadDeviceCommand;

	i2c_device_t slave =
	    {
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps};

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
		return kWarpStatusDeviceCommunicationFailed;

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
	int buffer_size = 0;

	while (true)
	{
		OSA_TimeDelay(200);

		uint8_t status_reg = 0;
		MUST(read_register(MMA8451Q_STATUS_REGISTER, 1, &status_reg));

		/* 6 lower bits. */
		const int readings_in_fifo = status_reg & 0b00111111;

		/* Nothing to do. */
		if (!readings_in_fifo)
			continue;

		const int to_read = MIN(MMA8451Q_FIFO_SIZE - buffer_size,
					readings_in_fifo);

		/*
		 * We have to empty the FIFO in a single transaction (data sheet
		 * says "It is assumed that the host application shall use the
		 * I2C multi-byte read transaction to empty the FIFO").
		 */
		MUST(read_register(MMA8451Q_FIFO_POINTER_REGISTER,
				   to_read * sizeof *buffer,
				   (uint8_t *)&(buffer[buffer_size])));

		buffer_size += to_read;

		/* Wait for a full buffer. */
		if (buffer_size < MMA8451Q_FIFO_SIZE)
			continue;

		if (buffer_size > MMA8451Q_FIFO_SIZE)
		{
			warpPrint("Corrupted buffer size %d\n", buffer_size);
			return;
		}

		/* Buffer is full. */
		for (int i = 0; i < buffer_size; ++i)
		{
			const struct Readings *readings = process_readings(&buffer[i]);
			warpPrint("%d,%d,%d\n",
				  readings->x,
				  readings->y,
				  readings->z);
		}

		/* Empty buffer. */
		buffer_size = 0;
	}
}
