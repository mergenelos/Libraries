/*
 * PCA96.c
 *
 *  Created on: Apr 20, 2022
 *      Author: mak
 */

#include "PCA96.h"
#include "math.h"


/*
 * Load up device default settings
 *
 * input: PCA96 struct instance
 * output: HAL_OK if no errors happen
 */
HAL_StatusTypeDef PCA96_Defaults(PCA96 *dev){
	dev->clock = 25; //MHz
	dev->updateRate = 200;
	dev->addr = PCA96_DEFAULT_ADDR;
	dev->memSize = I2C_MEMADD_SIZE_8BIT;
	dev->timeout = HAL_MAX_DELAY;
	return HAL_OK;
}

/*
 * A wrapper function to make working with HAL_I2C_Mem_Read easier
 *
 * input: PCA96 struct instance, Register address to be read,\
 * 		 A reference to a 8 bit variable to write retrieved register to
 *
 * 	output: returns the status of HAL_I2C_Mem_Read
 */
HAL_StatusTypeDef PCA96_ReadRegister(PCA96 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, dev->addr, reg, dev->memSize, data, 1, dev->timeout);

}

/*
 * A wrapper function to make working with HAL_I2C_Mem_Write easier
 *
 * input: PCA96 struct instance, Register address to be written,\
 * 		 A reference to a 8 bit variable to write into the register
 *
 * 	output: returns the status of HAL_I2C_Mem_Write
 */
HAL_StatusTypeDef PCA96_WriteRegister(PCA96 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, dev->addr, reg, dev->memSize, data, 1, dev->timeout);

}


/*
 * A wrapper function alter a single bit in a register
 *
 * input: PCA96 struct instance, Register address which the bit belongs to,\
 * 		 bit position in the register, value of the bit (ie. 0 or 1)
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_SetBit(PCA96 *dev, uint8_t reg, uint8_t bit, uint8_t value){

	uint8_t tmp;
	if(value) value = 1;

	if(HAL_OK != PCA96_ReadRegister(dev, reg, &tmp)){
		return HAL_ERROR;
	}

	tmp &= ~((1<<PCA96_MODE1_RESTART_BIT)|(1<<bit));
	tmp |= (value&1)<<bit;

	if(HAL_OK != PCA96_WriteRegister(dev, reg, &tmp)){
		return HAL_ERROR;
	}

	return HAL_OK;

}

/*
 * Software reset the passed device
 *
 * input: PCA96 struct instance
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_SoftwareReset(PCA96 *dev){

	uint8_t cmd = PCA96_SOFTWARE_RESET_ADDR;
	return HAL_I2C_Master_Transmit(dev->i2cHandle, dev->addr, &cmd, 1, 10);

}

/*
 * Put the passed device in sleep mode or wake up
 *
 * input: PCA96 struct instance , sleep bit value
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_Sleep(PCA96 *dev, uint8_t enable){

	return PCA96_SetBit(dev, PCA96_MODE1, PCA96_MODE1_SLEEP_BIT, enable);

}

/*
 * Set restart bit value
 *
 * input: PCA96 struct instance , restart bit value
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_Restart(PCA96 *dev, uint8_t enable){

	return PCA96_SetBit(dev, PCA96_MODE1, PCA96_MODE1_RESTART_BIT, enable);

}

/*
 * Enable / disable auto increment of the device register addresses
 *
 * input: PCA96 struct instance , auto increment bit value
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_AutoIncrement(PCA96 *dev, uint8_t enable){

	return PCA96_SetBit(dev, PCA96_MODE1, PCA96_MODE1_AI_BIT, enable);

}

/*
 * set subaddress
 *
 * input: PCA96 struct instance , subaddress register, enable/disable bit value
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_SubaddressRespond(PCA96 *dev, uint8_t subAddr, uint8_t enable){

	return PCA96_SetBit(dev, PCA96_MODE1, subAddr, enable);

}

/*
 * Enable / disable all call respond mode of PCA96
 *
 * input: PCA96 struct instance , enable/disable bit value
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_AllCallRespond(PCA96 *dev, uint8_t enable){

	return PCA96_SetBit(dev, PCA96_MODE1, PCA96_MODE1_ALLCALL_BIT, enable);

}

/*
 * Set the pre-scaler register value which affects the pwm frequency
 *
 * input: PCA96 struct instance , update rate in Hz <1526Hz and >24Hz
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_SetPwmFrequency(PCA96 *dev, uint8_t updateRate){

	uint8_t preScaleValue = (uint8_t)(((dev->clock*1000000)/(4096*updateRate)));
	if(preScaleValue < 0x03) preScaleValue = 0x03; //0x03 => 1526Hz, 0xFF => 24Hz
    if(preScaleValue > 0xFF) preScaleValue = 0xFF;


	PCA96_Sleep(dev, 1);
	uint8_t status = PCA96_WriteRegister(dev, PCA96_PRE_SCALE, &preScaleValue);

	if(status == HAL_OK) dev->updateRate = updateRate;

	PCA96_Sleep(dev, 0);
	PCA96_Restart(dev, 1);


	return status;

}

/*
 * Set pwm on and off time (pwm signal becomes on at OnTime and off at OffTime)
 *
 * input: PCA96 struct instance , pwm channel 0-15, OnTime, OffTime
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_SetPwm(PCA96 *dev, uint8_t channel, uint16_t onTime, uint16_t offTime){

	if(channel < 0 || channel > 15) return HAL_ERROR;

	uint8_t registerAddr;
	uint8_t msg[4];

	registerAddr = PCA96_LED0_ON_L + (4 * channel);
	msg[0] = onTime & 0xFF;
	msg[1] = onTime>>8;
	msg[2] = offTime & 0xFF;
	msg[3] = offTime>>8;

	return HAL_I2C_Mem_Write(dev->i2cHandle, dev->addr, registerAddr, 1, msg, 4, dev->timeout);

}

/*
 * wrapper to work with PCA96_SetPwm
 *
 * input: PCA96 struct instance , pwm channel 0-15, value of the pwm with onTime 0 /
 * 			and offTime of value
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_SetPin(PCA96 *dev, uint8_t channel, uint16_t value){

	if(value > 4095) value = 4095;

    if (value == 4095){
    	//signal on
    	return PCA96_SetPwm(dev, channel, 4096, 0);
    }
    else if (value == 0){
    	//signal off
    	return PCA96_SetPwm(dev, channel, 0, 4096);
    }else{
    	return PCA96_SetPwm(dev, channel, 0, value);
    }

}

/*
 * Initialize PCA96 with software reset, initial pwm frequency and auto increment
 *
 * input: PCA96 struct instance , i2c handle which the device is on
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA96_Init(PCA96 *dev, I2C_HandleTypeDef *i2cHandle){

	dev->i2cHandle = i2cHandle;

	PCA96_SoftwareReset(dev);
	PCA96_SetPwmFrequency(dev, dev->updateRate);
	PCA96_AutoIncrement(dev, 1);

	return HAL_OK;
}


/*
 * Basic use case example
 *
int main(void)
{

	.
	.
	.

  PCA96 dev;
  PCA96_Defaults(&dev);
  dev.update_rate = 50;

  PCA96_Init(&dev, &hi2c1);

  HAL_Delay(2000);

  while (1){

	  for (uint16_t Value = 50; Value < 570; Value++) {
		  PCA96_SetPin( &dev,  0, Value);
		  HAL_Delay(5);
	  }

	  HAL_Delay(500);

	  for (uint16_t Value = 570; Value > 50; Value--) {
		  PCA96_SetPin(&dev,  0, Value);
		  HAL_Delay(5);
	  }

	  HAL_Delay(500);

  }

	.
	.
	.

}
*
*/
