/*
 * HMC5883L.h
 *
 *  Created on: May 1, 2022
 *      Author: mak
 */

#ifndef INC_HMC5883_H_
#define INC_HMC5883_H_


#include "stm32f1xx_hal.h"

#define HMC5883_DEFAULT_ADDR  	0x3C		//0x1E << 1

#define HMC5883_CONF_REG_A		0x00		//Read/Write
#define HMC5883_CONF_REG_A_MASK	0x7F
	//Select number of samples averaged (1 to 8)
	//per measurement output.
	//MA10
	//00 = 1(Default);
	//01 = 2;
	//10 = 4;
	//11 = 8
	#define HMC5883_CONF_REG_A_MA1		0x40
	#define HMC5883_CONF_REG_A_MA0		0x20
	enum  HMC5883_samplesAveraged {
		HMC5883_SA_1 = 0x00,
		HMC5883_SA_2 = 0x20,
		HMC5883_SA_4 = 0x40,
		HMC5883_SA_8 = 0x60,
	};

	//Data Output Rate Bits. These bits set the rate at which data
	//is written to all three data output registers.
	//D210 - Typical Data Output Rate (Hz)
	//000 = 0.75Hz
	//001 = 1.5Hz
	//010 = 3Hz
	//011 = 7.5Hz
	//100 = 15Hz (Default)
	//101 = 30Hz
	//110 = 75Hz
	//111 = Reserved
	#define HMC5883_CONF_REG_A_DO2		0x10
	#define HMC5883_CONF_REG_A_DO1		0x08
	#define HMC5883_CONF_REG_A_DO0		0x04
	enum  HMC5883_dataRate {
		HMC5883_DR_0_75Hz 	= 	0x00,
		HMC5883_DR_1_5Hz 	= 	0x04,
		HMC5883_DR_3Hz 		= 	0x08,
		HMC5883_DR_7_5Hz 	= 	0x0C,
		HMC5883_DR_15Hz 	= 	0x10,
		HMC5883_DR_30Hz 	= 	0x14,
		HMC5883_DR_75Hz 	= 	0x18,
	};
	//Measurement Configuration Bits. These bits define the
	//measurement flow of the device, specifically whether or not
	//to incorporate an applied bias into the measurement.
	//MS10 - Measurement Mode
	//00 = Normal measurement configuration
	//01 = Positive bias configuration for X, Y, and Z axes.
	//10 = Negative bias configuration for X, Y and Z axes.
	//11 = This configuration is reserved.
	#define HMC5883_CONF_REG_A_MS1		0x02
	#define HMC5883_CONF_REG_A_MS0		0x01
	enum  HMC5883_measureConf {
		HMC5883_MC_NORMAL	= 	0x00,
		HMC5883_MC_PBIAS 	= 	0x01,
		HMC5883_MC_NBIAS 	= 	0x02,
	};

#define HMC5883_CONF_REG_B		0x01		//Read/Write
#define HMC5883_CONF_REG_B_MASK	0xE0
	//Gain Configuration Bits. These bits configure the gain for
	//the device. The gain configuration is common for all
	//channels.
	//G210 - Recommended Sensor Field Range - Gain - Digital Resolution	- Output Range
	//000 = ± 0.88 Ga	1370	0.73	0xF800–0x07FF(2048–2047 )
	//001 = ± 1.3 Ga	1090 	0.92	0xF800–0x07FF(2048–2047 )	//Default
	//010 = ± 1.9 Ga	820		1.22	0xF800–0x07FF(2048–2047 )
	//011 = ± 2.5 Ga	660		1.52	0xF800–0x07FF(2048–2047 )
	//100 = ± 4.0 Ga	440		2.27	0xF800–0x07FF(2048–2047 )
	//101 = ± 4.7 Ga	390		2.56	0xF800–0x07FF(2048–2047 )
	//110 = ± 5.6 Ga	330		3.03	0xF800–0x07FF(2048–2047 )
	//111 = ± 8.1 Ga	230		4.35	0xF800–0x07FF(2048–2047 )
	#define HMC5883_CONF_REG_B_GN2		0x80
	#define HMC5883_CONF_REG_B_GN1		0x40
	#define HMC5883_CONF_REG_B_GN0		0x20
	enum  HMC5883_gain {
		HMC5883_G_0_88	= 	0x00,
		HMC5883_G_1_3 	= 	0x20,
		HMC5883_G_1_9 	= 	0x40,
		HMC5883_G_2_5 	= 	0x60,
		HMC5883_G_4_0 	= 	0x80,
		HMC5883_G_4_7 	= 	0xA0,
		HMC5883_G_5_6 	= 	0xC0,
		HMC5883_G_8_1 	= 	0xE0,
	};

	//These bits must be cleared for correct operation.
	#define HMC5883_CONF_REG_B_CRBs		0x1F

#define HMC5883_MODE_REG		0x02		//Read/Write
#define HMC5883_MODE_REG_MASK	0x83
	//Set this pin to enable High Speed I2C, 3400kHz
	#define HMC5883_MODE_REG_HS0		0x80
	enum  HMC5883_i2cSpeed{
		HMC5883_i2c_HS	= 	0x80,
		HMC5883_i2c_LS	= 	0x00,
	};
	//Mode Select Bits. These bits select the
	//operation mode of this device.
	//MD10 - Operating Mode
	//00 = Continuous-Measurement Mode.
	//01 = Single-Measurement Mode (Default).
	//10 = Idle Mode. Device is placed in idle mode.
	//11 = Idle Mode. Device is placed in idle mode.
	#define HMC5883_MODE_REG_MR1		0x02
	#define HMC5883_MODE_REG_MR0		0x01
	enum  HMC5883_OPMode{
		HMC5883_OP_CONT		= 	0x00,
		HMC5883_OP_SINGLE 	= 	0x01,
		HMC5883_OP_IDLE		= 	0x02,
	};
//In the event the ADC reading overflows or underflows for the given channel,
//or if there is a math overflow during the bias measurement,
//this data registers will contain the value -4096.
//This register value will clear when after the next valid measurement is made

//When one or more of the output registers are read, new data cannot be placed in any of the output data registers until all
//six data output registers are read. This requirement also impacts DRDY and RDY, which cannot be cleared until new
//data is placed in all the output registers.

//The data output [X,Y,Z] registers are two 8-bit registers, data output register A and data output register B. These registers
//store the measurement result from channel [X,Y,Z]. Data output [X,Y,Z] register A contains the MSB from the measurement result,
//and data output [X,Y,Z] register B contains the LSB from the measurement result. The value stored in these two registers is a
//16-bit value in 2’s complement form, whose range is 0xF800 to 0x07FF.
#define HMC5883_X_MSB_REG		0x03		//Read
#define HMC5883_X_LSB_REG		0x04		//Read
#define HMC5883_Z_MSB_REG		0x05		//Read
#define HMC5883_Z_LSB_REG		0x06		//Read
#define HMC5883_Y_MSB_REG		0x07		//Read
#define HMC5883_Y_LSB_REG		0x08		//Read

#define HMC5883_STATUS_REG		0x09		//Read
#define HMC5883_STATUS_REG_MASK 0x03
	//Data output register lock. This bit is set when:
	//1.some but not all for of the six data output registers have
	//been read,
	//2. Mode register has been read.
	//When this bit is set, the six data output registers are locked
	//and any new data will not be placed in these register until
	//one of these conditions are met:
	//1.all six bytes have been read, 2. the mode register is
	//changed,
	//3. the measurement configuration (CRA) is changed,
	//4. power is reset.
	#define HMC5883_STATUS_REG_SR1		0x02

	//Ready Bit. Set when data is written to all six data registers.
	//Cleared when device initiates a write to the data output
	//registers and after one or more of the data output registers
	//are written to. When RDY bit is clear it shall remain cleared
	//for a 250us. DRDY pin can be used as an alternative to
	//the status register for monitoring the device for
	//measurement data
	#define HMC5883_STATUS_REG_SR0		0x01

#define HMC5883_IDENT_REG_A		0x10		//Read//Must contain 0x48
#define HMC5883_IDENT_REG_B		0x11		//Read//Must contain 0x34
#define HMC5883_IDENT_REG_C		0x12		//Read//Must contain 0x33


typedef struct{

	uint16_t X;	//The measurement result from channel X
	uint16_t Y;	//The measurement result from channel Y
	uint16_t Z;	//The measurement result from channel Z

	uint8_t samplesAveraged;	//MA1 - MA0
	uint8_t dataRate;			//DO2 - DO0
	uint8_t measurConf;			//MS1 - MS0
	uint8_t gainConf;			//GN2 - GN0
	uint8_t mode;				//MD1 - MD0

	I2C_HandleTypeDef  *i2cHandle; //i3c handle to talk to device
	uint8_t 			addr; 		//Device address on i2c buss

	uint32_t 			memSize; 	//Memory size of HMC5883 is 8 bits
	uint32_t 			timeout; 	//i2c timeout to talk to device

}HMC5883;

HAL_StatusTypeDef HMC5883_Defaults(HMC5883 *dev);
HAL_StatusTypeDef HMC5883_ReadRegister(HMC5883 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef HMC5883_ReadRegisters(HMC5883 *dev, uint8_t reg, uint8_t *data, uint8_t len);
HAL_StatusTypeDef HMC5883_WriteRegister(HMC5883 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef HMC5883_WriteCommand(HMC5883 *dev, uint8_t cmd);
HAL_StatusTypeDef HMC5883_ReadCommand(HMC5883 *dev, uint8_t *data, uint8_t len);
HAL_StatusTypeDef HMC5883_SetBit(HMC5883 *dev, uint8_t reg, uint8_t bit, uint8_t value);
HAL_StatusTypeDef HMC5883_Init(HMC5883 *dev, I2C_HandleTypeDef *i2cHandle);

HAL_StatusTypeDef HMC5883_ReadXYZ(HMC5883 *dev);
HAL_StatusTypeDef HMC5883_ReadXYZ_IT(HMC5883 *dev);

#endif /* INC_HMC5883_H_ */
