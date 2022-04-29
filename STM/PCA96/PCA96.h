/*
 * PCA96.h
 *
 *  Created on: Apr 20, 2022
 *      Author: mak
 */

#ifndef INC_PCA96_H_
#define INC_PCA96_H_

#include "stm32f1xx_hal.h"

//Registers in PCA96
#define PCA96_MODE1 				0x00

	#define PCA96_MODE1_RESTART_BIT			7	//0* Restart disabled.
												//1  Restart enabled. User writes logic 1 to this bit to clear it to logic 0.

	#define PCA96_MODE1_EXTCLK_BIT			6	//0* Use internal clock.
												//1  Use EXTCLK pin clock.

	#define PCA96_MODE1_AI_BIT				5	//0* Register Auto-Increment disabled.
												//1  Register Auto-Increment enabled.

	#define PCA96_MODE1_SLEEP_BIT			4	//0	 Normal mode.
												//1* Low power mode. Oscillator off.

	#define PCA96_MODE1_SUB1_BIT			3	//0* No response to I 2 C-bus subaddress 1.
	#define PCA96_MODE1_SUB2_BIT			2	//0* No response to I 2 C-bus subaddress 2.
	#define PCA96_MODE1_SUB3_BIT			1	//0* No response to I 2 C-bus subaddress 3.

	#define PCA96_MODE1_ALLCALL_BIT			0	//0	 Does not respond to LED All Call I 2C-bus address.
												//1* Responds to LED All Call I2 C-bus address.

#define PCA96_MODE2 				0x01

	#define PCA96_MODE2_INVRT_BIT			4	//0* Output logic state not inverted. Value to use when external driver used. Applicable when /OE = 0.
												//1  Output logic state inverted. Value to use when no external driver used.  Applicable when /OE = 0.

	#define PCA96_MODE2_OCH_BIT				3	//0* Outputs change on STOP command.
												//1  Outputs change on ACK.

	#define PCA96_MODE2_OUTDRV_BIT			2	//0  The 16 LEDn outputs are configured with an open-drain structure.
												//1* The 16 LEDn outputs are configured with a totem pole structure

	#define PCA96_MODE2_OUTNE_BIT_0			1	//00* When OE = 1 (output drivers not enabled), LEDn = 0.
	#define PCA96_MODE2_OUTNE_BIT_1			0	//01 When OE = 1 (output drivers not enabled):
												//LEDn = 1 when OUTDRV = 1
												//LEDn = high-impedance when OUTDRV = 0 (same as OUTNE[1:0] = 10)
												//1X When OE = 1 (output drivers not enabled), LEDn = high-impedance.


#define PCA96_SUBADR1 				0x02
#define PCA96_SUBADR2 				0x03
#define PCA96_SUBADR3 				0x04

#define PCA96_ALLCALLADR 			0x05
	#define PCA96_ALLCALLADR_LED_All_CALL_ADDR	0xE0	//Default value off ALLCALLADR register. [7:1]... [0] is reserved.

#define PCA96_LED0_ON_L 			0x06
#define PCA96_LED0_ON_H 			0x07
#define PCA96_LED0_OFF_L 			0x08
#define PCA96_LED0_OFF_H 			0x09
#define PCA96_LED1_ON_L 			0x0A
#define PCA96_LED1_ON_H 			0x0B
#define PCA96_LED1_OFF_L 			0x0C
#define PCA96_LED1_OFF_H 			0x0D
#define PCA96_LED2_ON_L 			0x0E
#define PCA96_LED2_ON_H 			0x0F
#define PCA96_LED2_OFF_L 			0x10
#define PCA96_LED2_OFF_H 			0x11
#define PCA96_LED3_ON_L 			0x12
#define PCA96_LED3_ON_H 			0x13
#define PCA96_LED3_OFF_L 			0x14
#define PCA96_LED3_OFF_H 			0x15
#define PCA96_LED4_ON_L 			0x16
#define PCA96_LED4_ON_H 			0x17
#define PCA96_LED4_OFF_L 			0x18
#define PCA96_LED4_OFF_H 			0x19
#define PCA96_LED5_ON_L 			0x1A
#define PCA96_LED5_ON_H 			0x1B
#define PCA96_LED5_OFF_L 			0x1C
#define PCA96_LED5_OFF_H 			0x1D
#define PCA96_LED6_ON_L 			0x1E
#define PCA96_LED6_ON_H 			0x1F
#define PCA96_LED6_OFF_L 			0x20
#define PCA96_LED6_OFF_H 			0x21
#define PCA96_LED7_ON_L 			0x22
#define PCA96_LED7_ON_H 			0x23
#define PCA96_LED7_OFF_L 			0x24
#define PCA96_LED7_OFF_H 			0x25
#define PCA96_LED8_ON_L 			0x26
#define PCA96_LED8_ON_H 			0x27
#define PCA96_LED8_OFF_L 			0x28
#define PCA96_LED8_OFF_H 			0x29
#define PCA96_LED9_ON_L 			0x2A
#define PCA96_LED9_ON_H 			0x2B
#define PCA96_LED9_OFF_L 			0x2C
#define PCA96_LED9_OFF_H 			0x2D
#define PCA96_LED10_ON_L 			0x2E
#define PCA96_LED10_ON_H 			0x2F
#define PCA96_LED10_OFF_L 			0x30
#define PCA96_LED10_OFF_H 			0x31
#define PCA96_LED11_ON_L 			0x32
#define PCA96_LED11_ON_H 			0x33
#define PCA96_LED11_OFF_L 			0x34
#define PCA96_LED11_OFF_H 			0x35
#define PCA96_LED12_ON_L 			0x36
#define PCA96_LED12_ON_H 			0x37
#define PCA96_LED12_OFF_L 			0x38
#define PCA96_LED12_OFF_H 			0x39
#define PCA96_LED13_ON_L 			0x3A
#define PCA96_LED13_ON_H 			0x3B
#define PCA96_LED13_OFF_L 			0x3C
#define PCA96_LED13_OFF_H 			0x3D
#define PCA96_LED14_ON_L 			0x3E
#define PCA96_LED14_ON_H 			0x3F
#define PCA96_LED14_OFF_L 			0x40
#define PCA96_LED14_OFF_H 			0x41
#define PCA96_LED15_ON_L 			0x42
#define PCA96_LED15_ON_H 			0x43
#define PCA96_LED15_OFF_L			0x44
#define PCA96_LED15_OFF_H 			0x45

#define	PCA96_ALL_LED_ON_L 			0xFA
#define	PCA96_ALL_LED_ON_H 			0xFB
#define PCA96_ALL_LED_OFF_L 		0xFC
#define PCA96_ALL_LED_OFF_H 		0xFD

	//R/W for LEDn_ and W for ALL_LED_
	#define PCA96_ALL_LED_LEDn_ON_L			0xFF	//LEDn_ON count for LED0, 8 LSBs
	#define PCA96_ALL_LED_LEDn_ON			0x10	//Turn drive on completely
	#define PCA96_ALL_LED_LEDn_ON_H			0x0F	//LEDn_ON count for LED0, 4 MSBs
	#define	PCA96_ALL_LED_LEDn_OFF_L		0xFF	//LEDn_OFF count for LED0, 8 LSBs
	#define PCA96_ALL_LED_LEDn_OFF			0x10	//Turn drive off completely
	#define PCA96_ALL_LED_LEDn_OFF_H		0x0F	//LEDn_OFF count for LED0, 4 MSBs

#define PCA96_PRE_SCALE 			0xFE		//Default is 1E = 200Hz
#define	PCA96_TestMode 				0xFF

#define PCA96_SOFTWARE_RESET_ADDR	0x06

#define PCA96_DEFAULT_ADDR			0x80


//Channels as they are marked on development boards
#define PCA96_channel0 				0
#define PCA96_channel1 				1
#define PCA96_channel2 				2
#define PCA96_channel3 				3
#define PCA96_channel4 				4
#define PCA96_channel5 				5
#define PCA96_channel6 				6
#define PCA96_channel7 				7
#define PCA96_channel8 				8
#define PCA96_channel9 				9
#define PCA96_channel10 			10
#define PCA96_channel11				11
#define PCA96_channel12				12
#define PCA96_channel13				13
#define PCA96_channel14				14
#define PCA96_channel15				15

//Simple struct to represent each PCA96 instance
typedef struct{

	uint8_t				clock; 		//Device clock rate in MHz
	uint8_t				updateRate; //Update rate set by Pre-scaler in Hz
	I2C_HandleTypeDef  *i2cHandle; 	//i3c handle to talk to device
	uint8_t 			addr; 		//Device address on i2c buss
	uint32_t 			memSize; 	//mem size of PCA96 is 8 bits
	uint32_t 			timeout; 	//i2c timeout to talk to device

}PCA96;


HAL_StatusTypeDef PCA96_Defaults(PCA96 *dev);
HAL_StatusTypeDef PCA96_ReadRegister(PCA96 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef PCA96_WriteRegister(PCA96 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef PCA96_SetBit(PCA96 *dev, uint8_t reg, uint8_t bit, uint8_t value);
HAL_StatusTypeDef PCA96_SoftwareReset(PCA96 *dev);
HAL_StatusTypeDef PCA96_Sleep(PCA96 *dev, uint8_t enable);
HAL_StatusTypeDef PCA96_Restart(PCA96 *dev, uint8_t enable);
HAL_StatusTypeDef PCA96_AutoIncrement(PCA96 *dev, uint8_t enable);
HAL_StatusTypeDef PCA96_SubaddressRespond(PCA96 *dev, uint8_t subAddr, uint8_t enable);
HAL_StatusTypeDef PCA96_AllCallRespond(PCA96 *dev, uint8_t enable);
HAL_StatusTypeDef PCA96_SetPwmFrequency(PCA96 *dev, uint8_t updateRate);
HAL_StatusTypeDef PCA96_SetPwm(PCA96 *dev, uint8_t channel, uint16_t onTime, uint16_t offTime);
HAL_StatusTypeDef PCA96_SetPin(PCA96 *dev, uint8_t channel, uint16_t value);
HAL_StatusTypeDef PCA96_Init(PCA96 *dev, I2C_HandleTypeDef *i2cHandle);

#endif /* INC_PCA96_H_ */
