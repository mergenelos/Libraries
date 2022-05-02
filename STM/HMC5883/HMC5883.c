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
 *  	Created on: May 1, 2022
 *  	Author: 	Murtaza Asaadi
 *  	License: 	GPL v.2
 *  	Github: 	https://github.com/makerlik
 *
 *  	Basic library to setup and communicate with HMC5883 three-axis digital compass
 */


#include "HMC5883.h"
#include "math.h"

/*
 * Load up device default settings
 *
 * input: HMC5883 struct instance
 * output: HAL_OK if no errors happen
 */
HAL_StatusTypeDef HMC5883_Defaults(HMC5883 *dev){

	dev->X = 0;
	dev->Y = 0;
	dev->Z = 0;

	dev->samplesAveraged = 0;
	dev->dataRate = 0;
	dev->measurConf = 0;
	dev->gainConf = 0;
	dev->mode = 0;

	dev->samplesAveraged = HMC5883_SA_1;
	dev->dataRate = HMC5883_DR_15Hz;
	dev->measurConf = HMC5883_MC_NORMAL;

	dev->gainConf = HMC5883_G_1_3;

	dev->mode = HMC5883_i2c_LS | HMC5883_OP_SINGLE;

	dev->addr = HMC5883_DEFAULT_ADDR;
	dev->memSize = I2C_MEMADD_SIZE_8BIT;
	dev->timeout = HAL_MAX_DELAY;

	return HAL_OK;

}

/*
 * A wrapper function to make working with HAL_I2C_Mem_Read easier
 *
 * input: HMC5883 struct instance, Register address to be read,\
 * 		 A reference to a 8 bit variable to write retrieved register to
 *
 * 	output: returns the status of HAL_I2C_Mem_Read
 */
HAL_StatusTypeDef HMC5883_ReadRegister(HMC5883 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, dev->addr, reg, dev->memSize, data, 1, dev->timeout);

}

/*
 * A wrapper function to make working with HAL_I2C_Mem_Read easier
 *
 * input: HMC5883 struct instance, Register address to be read,\
 * 		 A reference to a 8 bit variable to write retrieved register to,
 * 		 length of the data (number of registers to be read)
 *
 * 	output: returns the status of HAL_I2C_Mem_Read
 */
HAL_StatusTypeDef HMC5883_ReadRegisters(HMC5883 *dev, uint8_t reg, uint8_t *data, uint8_t len){

	return HAL_I2C_Mem_Read(dev->i2cHandle, dev->addr, reg, dev->memSize, data, len, dev->timeout);

}

/*
 * A wrapper function to make working with HAL_I2C_Mem_Write easier
 *
 * input: HMC5883 struct instance, Register address to be written,\
 * 		 A reference to a 8 bit variable to write into the register
 *
 * 	output: returns the status of HAL_I2C_Mem_Write
 */
HAL_StatusTypeDef HMC5883_WriteRegister(HMC5883 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, dev->addr, reg, dev->memSize, data, 1, dev->timeout);

}

/*
 * A wrapper function to make working with  HAL_I2C_Master_Transmit easier
 *
 * input: HMC5883 struct instance, Command to be written
 *
 * 	output: returns the status of HAL_I2C_Mem_Write
 */
HAL_StatusTypeDef HMC5883_WriteCommand(HMC5883 *dev, uint8_t cmd){

	uint8_t command = cmd;
	return HAL_I2C_Master_Transmit(dev->i2cHandle, dev->addr, &command, 1, dev->timeout);

}

/*
 * A wrapper function to make working with  HAL_I2C_Master_Receive easier
 *
 * input: HMC5883 struct instance, data will returned in this, data buffer size to be read
 *
 * 	output: returns the status of HAL_I2C_Mem_Write
 */
HAL_StatusTypeDef HMC5883_ReadCommand(HMC5883 *dev, uint8_t *data, uint8_t len){

	return HAL_I2C_Master_Receive(dev->i2cHandle, dev->addr, data, len, dev->timeout);

}

/*
 * Initialize HMC5883, Read and identify device, write configuration registers
 *
 * input: HMC5883 struct instance , i2c handle which the device is on
 *
 * 	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef HMC5883_Init(HMC5883 *dev, I2C_HandleTypeDef *i2cHandle){

	//Pass i2c handle to device instance
	dev->i2cHandle = i2cHandle;

	//Identify the device
	uint8_t ident = 0;
	HMC5883_ReadRegister(dev, HMC5883_IDENT_REG_A, &ident);
	if(ident != 0x48) return HAL_ERROR;
	HMC5883_ReadRegister(dev, HMC5883_IDENT_REG_B, &ident);
	if(ident != 0x34) return HAL_ERROR;
	HMC5883_ReadRegister(dev, HMC5883_IDENT_REG_C, &ident);
	if(ident != 0x33) return HAL_ERROR;

	//Setup reg A
	uint8_t cmd = 0;
	cmd = dev->samplesAveraged | dev->dataRate | dev->measurConf;
	HMC5883_WriteRegister(dev, HMC5883_CONF_REG_A, &cmd);

	//Setup reg B
	cmd = 0;
	cmd = dev->gainConf;
	HMC5883_WriteRegister(dev, HMC5883_CONF_REG_B, &cmd);

	//Setup MODE register
	cmd = 0;
	cmd = dev->mode;
	HMC5883_WriteRegister(dev, HMC5883_MODE_REG, &cmd);

	return HAL_OK;
}

/*
 * Get X,Y,Z values from HMC5883 without checking the status reg
 * can be used when using interrupts and DRDY pin
 *
 * input: HMC5883 struct instance
 *
 * output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef HMC5883_ReadXYZ_IT(HMC5883 *dev){

	//Read XZY , MSB and LSB regs in order
	uint8_t buff[6];
	HMC5883_ReadRegisters(dev, HMC5883_X_MSB_REG, buff, 6);

	//Get binary values from 2's complements
	for(int i = 0 ; i<6 ; i++){
		buff[i] = ~buff[i]+1;
	}

	//Load received values to device instance
	dev->X = buff[0] ;
	dev->X = dev->X << 8 | buff[1];

	dev->Y = buff[4] ;
	dev->Y = dev->Y << 8 | buff[5];

	dev->Z = buff[2] ;
	dev->Z = dev->Z << 8 | buff[3];

	return HAL_OK;

}

/*
 * Get X,Y,Z values from HMC5883 by pulling and
 * monitoring the STATUS register
 *
 * input: HMC5883 struct instance
 *
 * output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef HMC5883_ReadXYZ(HMC5883 *dev){

	//Check STATUS register for ready bit
	uint8_t cmd = dev->mode;
	HMC5883_WriteRegister(dev, HMC5883_MODE_REG, &cmd);
	HAL_Delay(100);
	uint8_t res = 0;
	HMC5883_ReadRegister(dev, HMC5883_STATUS_REG, &res);

	//read values and load to device instance
	if(res && HMC5883_STATUS_REG_SR0){
		HMC5883_ReadXYZ_IT(dev);
	}else{
		return HAL_BUSY;
	}

	return HAL_OK;

}











