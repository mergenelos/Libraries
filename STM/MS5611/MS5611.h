/*
 *
 *		 ███▄ ▄███▓ ▄▄▄       ██ ▄█▀▓█████  ██▀███   ██▓     ██▓ ██ ▄█▀
 *		▓██▒▀█▀ ██▒▒████▄     ██▄█▒ ▓█   ▀ ▓██ ▒ ██▒▓██▒    ▓██▒ ██▄█▒
 *		▓██    ▓██░▒██  ▀█▄  ▓███▄░ ▒███   ▓██ ░▄█ ▒▒██░    ▒██▒▓███▄░
 *		▒██    ▒██ ░██▄▄▄▄██ ▓██ █▄ ▒▓█  ▄ ▒██▀▀█▄  ▒██░    ░██░▓██ █▄
 *		▒██▒   ░██▒ ▓█   ▓██▒▒██▒ █▄░▒████▒░██▓ ▒██▒░██████▒░██░▒██▒ █▄
 *		░ ▒░   ░  ░ ▒▒   ▓▒█░▒ ▒▒ ▓▒░░ ▒░ ░░ ▒▓ ░▒▓░░ ▒░▓  ░░▓  ▒ ▒▒ ▓▒
 *		░  ░      ░  ▒   ▒▒ ░░ ░▒ ▒░ ░ ░  ░  ░▒ ░ ▒░░ ░ ▒  ░ ▒ ░░ ░▒ ▒░
 *		░      ░     ░   ▒   ░ ░░ ░    ░     ░░   ░   ░ ░    ▒ ░░ ░░ ░
 *			   ░         ░  ░░  ░      ░  ░   ░         ░  ░ ░  ░  ░
 *
 *
 *  	Created on:	Apr 26, 2022
 *  	Author: 	Murtaza Asaadi
 *  	License: 	GPL v.2
 *  	Github: 	https://github.com/makerlik
 *
 *  	Basic library to setup and communicate with HMC5883 three-axis digital compass
 */

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

#include "stm32f1xx_hal.h"

#define MS5611_DEFAULT_ADDR				0xEE		//0x77 << 1

/*
 * The reset can be sent at any time.
 * In the event that there is not a successful power on reset
 * this may be caused by the SDA being blocked by the module
 * in the acknowledge state. The only way to get the MS5611-01BA
 * to function is to send several SCLKs followed by a reset
 * sequence or to repeat power on reset.
*/
#define MS5611_RESET					0x1E

/*
 * OSR command values for pressure conversion
 * after sending one of these commands the pressure
 * raw value can be read as 24Bit from MS5611_ADC_READ
 * address.
 */
#define MS5611_CONVERT_D1_OSR_256		0x40
#define MS5611_CONVERT_D1_OSR_512		0x42
#define MS5611_CONVERT_D1_OSR_1024		0x44
#define MS5611_CONVERT_D1_OSR_2048		0x46
#define MS5611_CONVERT_D1_OSR_4096		0x48

/*
 * OSR command values for temperature conversion
 * after sending one of these commands the temperature
 * raw value can be read as 24Bit from MS5611_ADC_READ
 * address.
 */
#define MS5611_CONVERT_D2_OSR_256		0x50
#define MS5611_CONVERT_D2_OSR_512		0x52
#define MS5611_CONVERT_D2_OSR_1024		0x54
#define MS5611_CONVERT_D2_OSR_2048		0x56
#define MS5611_CONVERT_D2_OSR_4096		0x58

/*
 * Address of ADC register
 */
#define MS5611_ADC_READ					0x00

/*
 * Calibration values are stored here
 */
#define MS5611_PROM_READ				0xA0
	#define MS5611_PROM_READ_AD2			0x08
	#define MS5611_PROM_READ_AD1			0x04
	#define MS5611_PROM_READ_AD0			0x02

/*
 * Structure used to create a device instance of MS5611
 * with all the required configuration to run the device.
 * Raw and converted values for temperature and pressure
 * will be written here.
 */
typedef struct{

	uint16_t C1;	//Pressure sensitivity | SENST1
	uint16_t C2;	//Pressure offset | OFFT1
	uint16_t C3;	//Temperature coefficient of pressure sensitivity | TCS
	uint16_t C4;	//Temperature coefficient of pressure offset | TCO
	uint16_t C5;	//Reference temperature | TREF
	uint16_t C6;	//Temperature coefficient of the temperature | TEMPSENS

	int D1;			//Digital pressure value
	int D2;			//Digital temperature value

	uint8_t D1_OSR;	//OSR value of pressure
	uint8_t D2_OSR;	//OSR value of temperature

	float pressure;	//Pressure value after conversion
	float temp;		//Temperature value after conversion

	I2C_HandleTypeDef  *i2cHandle;	//i3c handle to talk to device
	uint8_t 			addr; 		//Device address on i2c buss

	uint32_t 			memSize; 	//mem size of MS5611 is 8 bits
	uint32_t 			timeout; 	//i2c timeout to talk to device

}MS5611;

/*
 * Basic functions to work with i2c
 */
HAL_StatusTypeDef MS5611_ReadRegister(MS5611 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MS5611_ReadRegisters(MS5611 *dev, uint8_t reg, uint8_t *data, uint8_t len);
HAL_StatusTypeDef MS5611_WriteRegister(MS5611 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MS5611_WriteCommand(MS5611 *dev, uint8_t cmd);
HAL_StatusTypeDef MS5611_ReadCommand(MS5611 *dev, uint8_t *data, uint8_t len);
HAL_StatusTypeDef MS5611_SetBit(MS5611 *dev, uint8_t reg, uint8_t bit, uint8_t value);

/*
 * MS5611 specific functions
 */
HAL_StatusTypeDef MS5611_Defaults(MS5611 *dev);
HAL_StatusTypeDef MS5611_Reset(MS5611 *dev);
HAL_StatusTypeDef MS5611_Init(MS5611 *dev, I2C_HandleTypeDef *i2cHandle);
int MS5611_GetTemp(MS5611 *dev);
int MS5611_GetPressure(MS5611 *dev);

#endif /* INC_MS5611_H_ */
