/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.
	2023 Antros Economides.

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

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "devINA219.h"
#include "fsl_i2c_master_driver.h"

#include <string.h>

#define INA219_REGISTER_POINTER_MIN 0x00
#define INA219_REGISTER_POINTER_MAX 0x05

#define INA219_REGISTER_CONFIGURATION 0x00
#define INA219_REGISTER_SHUNT_VOLTAGE 0x01
#define INA219_REGISTER_BUS_VOLTAGE 0x02
#define INA219_REGISTER_POWER 0x03
#define INA219_REGISTER_CURRENT 0x04
#define INA219_REGISTER_CALIBRATION 0x05

#define INA219_REGISTER_SIZE 2 /* bytes */

extern volatile WarpI2CDeviceState deviceINA219State;
extern volatile uint32_t gWarpI2cBaudRateKbps;
extern volatile uint32_t gWarpI2cTimeoutMilliseconds;

void initINA219(const uint8_t i2c_address)
{
	deviceINA219State.i2cAddress = i2c_address;

	memset((void *)deviceINA219State.i2cBuffer,
	       0,
	       sizeof deviceINA219State.i2cBuffer);

	/* External device; we don't control its power supply. */
	deviceINA219State.operatingVoltageMillivolts = -1;
}

static bool
is_invalid_register(const uint8_t register_address)
{
	return register_address < INA219_REGISTER_POINTER_MIN ||
	       register_address > INA219_REGISTER_POINTER_MAX;
}

static WarpStatus
writeSensorRegisterINA219(const uint8_t device_register, const uint16_t *payload)
{
	/* Check register address is valid. */
	if (is_invalid_register(device_register))
		return kWarpStatusBadDeviceCommand;

	const i2c_device_t slave = {
	    .address = deviceINA219State.i2cAddress,
	    .baudRate_kbps = gWarpI2cBaudRateKbps,
	};

	/* Construct optional payload. */
	uint8_t payload_bytes[2];
	if (payload)
	{
		/* MSB is sent first. */
		payload_bytes[0] = *payload >> 8;
		payload_bytes[1] = *payload & 0xff;
	}

	warpEnableI2Cpins();

	const i2c_status_t status = I2C_DRV_MasterSendDataBlocking(
	    0 /* I2C instance */,
	    &slave,
	    &device_register,
	    1,
	    payload ? payload_bytes : NULL,
	    payload ? sizeof payload_bytes : 0,
	    gWarpI2cTimeoutMilliseconds);

	return status == kStatus_I2C_Success ? kWarpStatusOK
					     : kWarpStatusDeviceCommunicationFailed;
}

static WarpStatus
writeRegisterPointerINA219(const uint8_t device_register)
{
	return writeSensorRegisterINA219(device_register, NULL);
}

WarpStatus
configureSensorINA219(void)
{
	/*
	 * Configuration register:
	 * Reset = false				0
	 * Unused					0
	 * Bus Voltage Range = 16 V     		0
	 * PGA = gain 1, +- 40 mV       		00
	 * Bus ADC Resolution = 12-bits, no averaging	0011
	 * Shunt ADC Resolution = 12-bits, no averaging	0011
	 * Mode = shunt and bus, continuous		111
	 */
	const uint16_t CONFIGURATION_REGISTER_VALUE = 0x019F;
	const WarpStatus write_status_1 = writeSensorRegisterINA219(INA219_REGISTER_CONFIGURATION,
								    &CONFIGURATION_REGISTER_VALUE);

	/* Calibration register: 10 μA per bit. */
	const uint16_t CALIBRATION_REGISTER_VALUE = 0xA000;
	const WarpStatus write_status_2 = writeSensorRegisterINA219(INA219_REGISTER_CALIBRATION,
								    &CALIBRATION_REGISTER_VALUE);

	const WarpStatus write_status_3 = writeRegisterPointerINA219(INA219_REGISTER_CURRENT);

	return write_status_1 | write_status_2 | write_status_3;
}

static WarpStatus
readSensorRegisterINA219(void)
{
	/* All registers are 2 bytes long. */
	_Static_assert(sizeof deviceINA219State.i2cBuffer >= INA219_REGISTER_SIZE);

	const i2c_device_t slave = {
	    .address = deviceINA219State.i2cAddress,
	    .baudRate_kbps = gWarpI2cBaudRateKbps,
	};

	warpEnableI2Cpins();

	/*
	 * No command; the register pointer should be set in a previous
	 * transaction.
	 */
	const i2c_status_t status = I2C_DRV_MasterReceiveDataBlocking(
	    0 /* I2C peripheral instance */,
	    &slave,
	    NULL,
	    0,
	    (uint8_t *)deviceINA219State.i2cBuffer,
	    INA219_REGISTER_SIZE,
	    gWarpI2cTimeoutMilliseconds);

	return status == kStatus_I2C_Success ? kWarpStatusOK
					     : kWarpStatusDeviceCommunicationFailed;
}

static uint16_t
convertRegisterValueINA219(const uint8_t device_register, uint16_t value)
{
	switch (device_register)
	{
	case INA219_REGISTER_CONFIGURATION:
		return value;
	case INA219_REGISTER_SHUNT_VOLTAGE:
		/* LSB = 10 uV. */
		return value * 10;
	case INA219_REGISTER_BUS_VOLTAGE:
		/*
		 * LSB = 4 mV. Voltage is stored in bits 14:3.
		 * Bit 0 signifies overflow.
		 */
		return value & 1 ? 0 : (value >> 3) * 4;
	case INA219_REGISTER_POWER:
		/* LSB = 200 uW (configuration-dependent). */
		return value * 200;
	case INA219_REGISTER_CURRENT:
		/* LSB = 10 uA (configuration-dependent). */
		return value * 10;
	case INA219_REGISTER_CALIBRATION:
		return value;
	}

	return -1;
}

static void
printRegisterINA219(const uint8_t device_register, bool hexModeFlag)
{
	WarpStatus i2c_status = kWarpStatusOK;

	i2c_status |= writeRegisterPointerINA219(device_register);
	i2c_status |= readSensorRegisterINA219();

	const uint16_t read_value = (deviceINA219State.i2cBuffer[0] << 8) |
				    (deviceINA219State.i2cBuffer[1]);

	if (i2c_status != kWarpStatusOK)
		warpPrint(" ----,");
	else if (hexModeFlag)
		warpPrint(" 0x%04x,", read_value);
	else
		warpPrint(" %d,", convertRegisterValueINA219(device_register, read_value));
}

void printSensorDataINA219(bool hexModeFlag)
{
	printRegisterINA219(INA219_REGISTER_CONFIGURATION, hexModeFlag);
	printRegisterINA219(INA219_REGISTER_SHUNT_VOLTAGE, hexModeFlag);
	printRegisterINA219(INA219_REGISTER_BUS_VOLTAGE, hexModeFlag);
	printRegisterINA219(INA219_REGISTER_POWER, hexModeFlag);
	printRegisterINA219(INA219_REGISTER_CURRENT, hexModeFlag);
	printRegisterINA219(INA219_REGISTER_CALIBRATION, hexModeFlag);
}
