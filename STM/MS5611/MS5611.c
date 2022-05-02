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

#include "MS5611.h"
#include "math.h"

/*
 * Load up device default settings
 *
 *	input: MS5611 struct instance
 *
 *	output: HAL_OK if no errors happen
 */
HAL_StatusTypeDef MS5611_Defaults(MS5611 *dev){

	dev->C1 = 0;
	dev->C2 = 0;
	dev->C3 = 0;
	dev->C4 = 0;
	dev->C5 = 0;
	dev->C6 = 0;

	dev->D1 = 0;
	dev->D2 = 0;

	dev->D1_OSR = MS5611_CONVERT_D1_OSR_4096;
	dev->D2_OSR = MS5611_CONVERT_D2_OSR_4096;

	dev->addr = MS5611_DEFAULT_ADDR;
	dev->memSize = I2C_MEMADD_SIZE_8BIT;
	dev->timeout = HAL_MAX_DELAY;

	return HAL_OK;

}

/*
 * A wrapper function to make working with HAL_I2C_Mem_Read easier
 *
 *	input: MS5611 struct instance, Register address to be read,\
 *		A reference to a 8 bit variable to write retrieved register to
 *
 *	output: returns the status of HAL_I2C_Mem_Read
 */
HAL_StatusTypeDef MS5611_ReadRegister(MS5611 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, dev->addr, reg, dev->memSize, data, 1, dev->timeout);

}

/*
 * A wrapper function to make working with HAL_I2C_Mem_Read easier
 *
 *	input: MS5611 struct instance, Register address to be read,\
 *		A reference to a 8 bit variable to write retrieved register to,
 *		length of the data (number of registers to be read)
 *
 *	output: returns the status of HAL_I2C_Mem_Read
 */
HAL_StatusTypeDef MS5611_ReadRegisters(MS5611 *dev, uint8_t reg, uint8_t *data, uint8_t len){

	return HAL_I2C_Mem_Read(dev->i2cHandle, dev->addr, reg, dev->memSize, data, len, dev->timeout);

}

/*
 * A wrapper function to make working with HAL_I2C_Mem_Write easier
 *
 *	input: MS5611 struct instance, Register address to be written,\
 *		A reference to a 8 bit variable to write into the register
 *
 *	output: returns the status of HAL_I2C_Mem_Write
 */
HAL_StatusTypeDef MS5611_WriteRegister(MS5611 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, dev->addr, reg, dev->memSize, data, 1, dev->timeout);

}

/*
 * A wrapper function to make working with  HAL_I2C_Master_Transmit easier
 *
 *	input: MS5611 struct instance, Command to be written
 *
 *	output: returns the status of HAL_I2C_Mem_Write
 */
HAL_StatusTypeDef MS5611_WriteCommand(MS5611 *dev, uint8_t cmd){

	uint8_t command = cmd;
	return HAL_I2C_Master_Transmit(dev->i2cHandle, dev->addr, &command, 1, dev->timeout);

}

/*
 * A wrapper function to make working with  HAL_I2C_Master_Receive easier
 *
 *	input: MS5611 struct instance, data will returned in this, data buffer size to be read
 *
 *	output: returns the status of HAL_I2C_Mem_Write
 */
HAL_StatusTypeDef MS5611_ReadCommand(MS5611 *dev, uint8_t *data, uint8_t len){

	return HAL_I2C_Master_Receive(dev->i2cHandle, dev->addr, data, len, dev->timeout);

}
/*
 * Set restart bit value
 *
 *	input: MS5611 struct instance , restart bit value
 *
 *	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef MS5611_Reset(MS5611 *dev){

	return MS5611_WriteCommand(dev, MS5611_RESET);

}

/*
 * Initialize MS5611 with reset, resets the sensor and reads the calibration values
 *
 *	input: MS5611 struct instance , i2c handle which the device is on
 *
 *	output: returns the status based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef MS5611_Init(MS5611 *dev, I2C_HandleTypeDef *i2cHandle){

	//Pass i2c handle to device instance
	dev->i2cHandle = i2cHandle;

	//Reset the device and wait proper amount (more than 15ms)
	MS5611_Reset(dev);
	HAL_Delay(50);

	//Read calibration bytes from the device and store them in device instance
	uint16_t calibData[6];
	uint8_t buff[2];
    for (int i=0;i<6;i++){
    	MS5611_WriteCommand(dev,MS5611_PROM_READ+(i+1)*2);
    	HAL_Delay(1);
    	MS5611_ReadCommand(dev, buff, sizeof(buff));
    	HAL_Delay(1);
    	calibData[i] = buff[0]<<8|buff[1];
    }

    dev->C1 = calibData[0];
    dev->C2 = calibData[1];
    dev->C3 = calibData[2];
    dev->C4 = calibData[3];
    dev->C5 = calibData[4];
    dev->C6 = calibData[5];

	return HAL_OK;
}


/*
 * Get the temperature value from the device and convert it
 *
 *	input: MS5611 struct instance
 *
 *	output: returns int value (with out float conversion) of
 *			temperature
 */
int MS5611_GetTemp(MS5611 *dev){

	//Start recording temperature value
    MS5611_WriteCommand(dev, dev->D2_OSR);
    HAL_Delay(250);

    //Read ADC output value and store it in D2 variable of Device instance
    uint8_t buff[3];
    MS5611_ReadRegisters(dev, MS5611_ADC_READ, buff, 3);

    dev->D2 = 0;
    dev->D2 = dev->D2<<8 | buff[0];
    dev->D2 = dev->D2<<8 | buff[1];
    dev->D2 = dev->D2<<8 | buff[2];

    //P.8 of datasheet. Convert raw value to Temperature
    int dT = dev->D2 - ((int)dev->C5 << 8);
    int tmp = (2000 + (((int64_t)dT * (int64_t)dev->C6) >> 23));
    int	T2;

    //P.9 of datasheet. SECOND ORDER TEMPERATURE COMPENSATION
    if (tmp<2000){
      T2=pow(dT,2)/2147483648;
    }else{
          T2=0;
     }

    //Calculate compensated temperature
	int temp = ((2000 + (((int64_t)dT * (int64_t)dev->C6) >> 23))-T2);

	//Write temperature to device instance temp variable
	dev->temp = (float)temp/100;

	return temp;

}

/*
 * Get the pressure value from the device and convert it
 *
 *	input: MS5611 struct instance
 *
 *	output: returns int value (with out float conversion) of
 *			pressure
 */
int MS5611_GetPressure(MS5611 *dev){

	int64_t OFF, OFF2, SENS, SENS2;
	int dT, temp, T2, pressure;
	uint8_t buff[3];

	//Start recording pressure value
    MS5611_WriteCommand(dev, dev->D1_OSR);
    HAL_Delay(250);

    //Read ADC output value and store it in D1 variable of Device instance
    MS5611_ReadRegisters(dev, MS5611_ADC_READ, buff, 3);

    dev->D1 = 0;
    dev->D1 = dev->D1<<8 | buff[0];
    dev->D1 = dev->D1<<8 | buff[1];
    dev->D1 = dev->D1<<8 | buff[2];

    //Start recording temperature value
    MS5611_WriteCommand(dev, dev->D2_OSR);
    HAL_Delay(250);

    //Read ADC output value and store it in D2 variable of Device instance
    buff[0]=0,buff[1]=0,buff[2]=0;
    MS5611_ReadRegisters(dev, MS5611_ADC_READ, buff, 3);

    dev->D2 = 0;
    dev->D2 = dev->D2<<8 | buff[0];
    dev->D2 = dev->D2<<8 | buff[1];
    dev->D2 = dev->D2<<8 | buff[2];

    //P.8 of datasheet. Convert raw value to Temperature
    dT = dev->D2 - ((int)dev->C5 << 8);
    int tmp = (2000 + (((int64_t)dT * (int64_t)dev->C6) >> 23));

    //P9 of datasheet. SECOND ORDER TEMPERATURE COMPENSATION
    if(tmp<2000){
    	T2=pow(dT,2)/2147483648;
    	OFF2=5*pow((tmp-2000),2)/2;
    	SENS2=5*pow((tmp-2000),2)/4;
    	if(tmp<-1500){
    		OFF2=OFF2+7*pow((tmp+1500),2);
    		SENS2=SENS2+11*pow((tmp+1500),2)/2;
    	}
	}
	else{
		T2=0;
		OFF2=0;
		SENS2=0;
	}

    //Calculate compensated pressure value and temperature
    temp = ((2000 + (((int64_t)dT * (int64_t)dev->C6) >> 23))-T2);
	OFF = (((unsigned int)dev->C2 << 16) + (((int64_t)dev->C4 * dT) >> 7)-OFF2);
	SENS = (((unsigned int)dev->C1 << 15) + (((int64_t)dev->C3 * dT) >> 8)-SENS2);
	pressure = (((dev->D1 * SENS) >> 21) - OFF) >> 15;

	//Write temperature to device instance temp variable
	dev->temp = (float)temp/100;
	//Write pressure to device instance pressure variable
	dev->pressure = (float)pressure/100;

	return pressure;

}
