//#include "Projector.h"
#include "I2C_Device.h"
#include <unistd.h>
//class I2C_Device
//class Projector
//constexpr int PROJECTOR_SLAVE_ADDRESS          = 0x1a;
//constexpr int I2C0_PORT         = 0; // I2C0
I2C_Device projectorI2cDevice(0x1a,0);
const I_I2C_Device& _i2cDevice=projectorI2cDevice;
constexpr int PROJECTOR_LED_ENABLE_REG         = 0x10;
constexpr int PROJECTOR_DISABLE_LEDS           = 0x0;
constexpr int PROJECTOR_ENABLE_LEDS            = 0x7;
constexpr int PROJECTOR_WRITE_BIT             = 0x80;
int main(int argc, char** argv)
{
	I2C_Device projectorI2cDevice(0x1a,0);
	//I2C_Device projectorI2cDevice(PROJECTOR_SLAVE_ADDRESS,I2C0_PORT);
//	Projector projector(projectorI2cDevice);
	//I2CWrite(PROJECTOR_LED_ENABLE_REG, PROJECTOR_DISABLE_LEDS);
	//I2CWrite(PROJECTOR_LED_ENABLE_REG, PROJECTOR_ENABLE_LEDS);
	_i2cDevice.Write(0x10 | PROJECTOR_WRITE_BIT, 0x7);
	//_i2cDevice.Write(registerAddress | PROJECTOR_WRITE_BIT, data);
	return 1;
}
