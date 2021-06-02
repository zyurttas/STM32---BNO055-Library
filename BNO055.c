/*
 * BNO055.c
 *
 *  Created on: Feb 4, 2021
 *      Author: Zeynep Yurttaþ
 */


#include "BNO055.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>
#include "stdio.h"


extern I2C_HandleTypeDef hi2c3;
#define i2cHandler hi2c3

/******************************************READ-WRITE FUNCTIONS****************************************/

static BNO055_STATUS bus_read(uint8_t reg_addr,uint8_t *reg_data, uint8_t r_len){

	uint8_t i2cBuf[2];
	i2cBuf[0] = reg_addr;

	if(HAL_I2C_Master_Transmit(&i2cHandler, BNO055_I2C_ADDR1<<1, i2cBuf, 1, 10) != HAL_OK){
		return BNO055_ERROR;
	}

	if(HAL_I2C_Master_Receive(&i2cHandler, BNO055_I2C_ADDR1<<1, reg_data, r_len, BNO055_TIMEOUT) == HAL_OK){
		return BNO055_SUCCESS;
	}
	else{
		return BNO055_ERROR;
	}
}

static BNO055_STATUS Vector_Read(uint8_t reg_addr,int16_t *reg_data, uint8_t len){

	if(HAL_I2C_Master_Transmit(&i2cHandler, BNO055_I2C_ADDR1 << 1, &reg_addr, 1, BNO055_TIMEOUT) != HAL_OK){
		return BNO055_ERROR;
	}

	reg_data[0] = reg_addr;

	if(HAL_I2C_Master_Receive(&i2cHandler, BNO055_I2C_ADDR1 << 1, (uint8_t *) reg_data, len, BNO055_TIMEOUT) == HAL_OK){
		return BNO055_SUCCESS;
	}
	else{
		return BNO055_ERROR;
	}
}

static BNO055_STATUS bus_write(uint8_t reg_addr, uint8_t reg_data){

	uint8_t i2cData[2];
	i2cData[0] = reg_addr;
	i2cData[1] = reg_data;
	if(HAL_I2C_Master_Transmit(&i2cHandler, BNO055_I2C_ADDR1<<1, i2cData, 2, BNO055_TIMEOUT) == HAL_OK){
		return BNO055_SUCCESS;
	}
	else{
		return BNO055_ERROR;
	}
}
/******************************END OF READ-WRITE FUNCTIONS*******************************************/
/* write page id as zero
 * take a built in self test
 * set operation mode as NDOF
 */
BNO055_STATUS BNO055_Init(bno055_t *bno055){

	/* Set the device address as default i2c address */
	bno055->system.dev_addr = BNO055_I2C_ADDR1;

	while (bno055->system.chip_id != BNO055_CHIP_ID) {
		HAL_Delay(650); //LIMIT THE TIME IT SHOULD WAIT
		if (bus_read(BNO055_CHIP_ID_ADDR, &bno055->system.chip_id, ONE_BYTE) != BNO055_SUCCESS) {
			bno055->system.sysError = BNO055_ERROR;
		}
	}

	/*Set Page ID as zero */
	bno055->system.page_id = BNO055_PAGE_ZERO;
	if(bus_write(BNO055_PAGE_ID_ADDR, bno055->system.page_id) != BNO055_SUCCESS )
		return BNO055_ERROR;

	/*Set the operation mode to configuration mode*/
	bno055->system.opMode = OPERATION_MODE_CONFIG;
	if(BNO055_SetOpMode(bno055) != BNO055_SUCCESS )
		return BNO055_ERROR;

	/* Power on Reset */
	if (bus_write(BNO055_SYS_TRIGGER_ADDR, RST_SYS_ENABLE) != BNO055_SUCCESS) {
		return BNO055_ERROR;
	}

	HAL_Delay(650);

	while (bno055->system.chip_id != BNO055_CHIP_ID) { //LIMIT THE TIME IT SHOULD WAIT
		bus_read(BNO055_CHIP_ID_ADDR, &bno055->system.chip_id, ONE_BYTE);
	}

	bno055->offset.accel_offset[0] = -26;
	bno055->offset.accel_offset[1] = 12;
	bno055->offset.accel_offset[2] = -18;

	bno055->offset.gyro_offset[0] = -4;
	bno055->offset.gyro_offset[1] = -18;
	bno055->offset.gyro_offset[2] = 2;

	bno055->offset.mag_offset[0] = 339;
	bno055->offset.mag_offset[1] = -94;
	bno055->offset.mag_offset[2] = -656;

	bno055->offset.acc_radius =	1000;
	bno055->offset.mag_radius = 400;
	if(BNO055_SetCalibrationOffset(bno055) != BNO055_SUCCESS)
		return BNO055_ERROR;

	if (bus_write(BNO055_OPR_MODE_ADDR, POWER_MODE_NORMAL) != BNO055_SUCCESS)
		return BNO055_ERROR;

	/* Trigger a BIST and get the self-test results */
	if(BNO055_BuildInSelfTest(bno055) != BNO055_SUCCESS )
		return BNO055_ERROR;

	/* Set the operation mode to NDOF ***END OF CONFIG MODE*** */
	bno055->system.opMode = OPERATION_MODE_NDOF;
	if(BNO055_SetOpMode(bno055) != BNO055_SUCCESS )
		return BNO055_ERROR;

	return BNO055_SUCCESS;
}

BNO055_STATUS BNO055_SetOpMode(bno055_t *bno055){

	return bus_write(BNO055_OPR_MODE_ADDR, bno055->system.opMode);

}

/* trigger a built in self test */
BNO055_STATUS BNO055_BuildInSelfTest(bno055_t *bno055){

	uint8_t setBit = 0x01;
	uint8_t resetBit = 0x00;

	// Set bit 0 of SYS_TRIGGER register to trigger a self test
	if(bus_write(BNO055_SYS_TRIGGER_ADDR, setBit) != BNO055_SUCCESS )
		return BNO055_ERROR;

	HAL_Delay(400);
	// Read self-test result
	if(bus_read(BNO055_SELFTEST_RESULT_ADDR, &(bno055->system.stResult), ONE_BYTE) != BNO055_SUCCESS )
		return BNO055_ERROR;

	// Reset bit 0 of SYS_TRIGGER register to untrigger self test
	if(bus_write(BNO055_SYS_TRIGGER_ADDR, resetBit) != BNO055_SUCCESS)
		return BNO055_ERROR;

	if((bno055->system.stResult & 0x0F) != 0x0F){
		return BNO055_ERROR;
	}

	return BNO055_SUCCESS;
}

/* read system status, if there is an error, check bno055->sysError */
BNO055_STATUS BNO055_SystemStatus(bno055_t *bno055){

	if(bus_read(BNO055_SYS_STAT_ADDR, &(bno055->system.sysStatus), ONE_BYTE) != BNO055_SUCCESS)
		return BNO055_ERROR;

	/*Read: 0 System idle,
	 *1 System Error,
	 *2 Initializing peripherals
	 *3 System Initialization
	 *4 Executing selftest,
	 *5 Sensor fusion algorithm running,
	 *6 System running without fusion algorithm*/

	if((bno055->system.sysStatus == 0x01) && (bus_read(BNO055_SYS_ERR_ADDR, &(bno055->system.sysError), ONE_BYTE) != BNO055_SUCCESS ))
		return BNO055_ERROR;

	/*Read : 0 No error
	 *1 Peripheral initialization error
	 *2 System initialization error
	 *3 Self test result failed
	 *4 Register map value out of range
	 *5 Register map address out of range
	 *6 Register map write error
	 *7 BNO low power mode not available for selected operation mode
	 *8 Accelerometer power mode not available
	 *9 Fusion algorithm configuration error
	 *A Sensor configuration error*/

	return BNO055_SUCCESS;
}

/* calibration values must be 3 for full calibration
 * This function won't stop until the sensor is completely calibrated. */

BNO055_STATUS BNO055_CalibrationStatus(bno055_t *bno055){

	while((bno055->calibration.sys != 0x03)||(bno055->calibration.gyro != 0x03)||(bno055->calibration.accel != 0x03)||(bno055->calibration.mag != 0x03)){
		if(bus_read(BNO055_CALIB_STAT_ADDR, &(bno055->calibration.data), ONE_BYTE) != BNO055_SUCCESS )
			return BNO055_ERROR;

		bno055->calibration.sys = (bno055->calibration.data >> 6) & 0x03;
		bno055->calibration.gyro = (bno055->calibration.data >> 4) & 0x03;
		bno055->calibration.accel = (bno055->calibration.data >> 2) & 0x03;
		bno055->calibration.mag = bno055->calibration.data & 0x03;


	}

	return BNO055_SUCCESS;
}

/* Use this function only in config mode */
BNO055_STATUS BNO055_SetCalibrationOffset(bno055_t *bno055){

	if(bus_write(BNO055_ACCEL_OFFSET_X_LSB_ADDR, bno055->offset.accel_offset[0]) != BNO055_SUCCESS)
		return BNO055_ERROR;
	if(bus_write(BNO055_ACCEL_OFFSET_Y_LSB_ADDR, bno055->offset.accel_offset[1]) != BNO055_SUCCESS)
		return BNO055_ERROR;
	if(bus_write(BNO055_ACCEL_OFFSET_Z_LSB_ADDR, bno055->offset.accel_offset[2]) != BNO055_SUCCESS)
		return BNO055_ERROR;

	if(bus_write(BNO055_MAG_OFFSET_X_LSB_ADDR, bno055->offset.mag_offset[0]) != BNO055_SUCCESS)
		return BNO055_ERROR;
	if(bus_write(BNO055_MAG_OFFSET_Y_LSB_ADDR, bno055->offset.mag_offset[1]) != BNO055_SUCCESS)
		return BNO055_ERROR;
	if(bus_write(BNO055_MAG_OFFSET_Z_LSB_ADDR, bno055->offset.mag_offset[2]) != BNO055_SUCCESS)
		return BNO055_ERROR;

	if(bus_write(BNO055_GYRO_OFFSET_X_LSB_ADDR, bno055->offset.gyro_offset[0]) != BNO055_SUCCESS)
		return BNO055_ERROR;
	if(bus_write(BNO055_GYRO_OFFSET_Y_LSB_ADDR, bno055->offset.gyro_offset[1]) != BNO055_SUCCESS)
		return BNO055_ERROR;
	if(bus_write(BNO055_GYRO_OFFSET_Z_LSB_ADDR, bno055->offset.gyro_offset[2]) != BNO055_SUCCESS)
		return BNO055_ERROR;

	if(bus_write(BNO055_ACCEL_RADIUS_LSB_ADDR, bno055->offset.acc_radius) != BNO055_SUCCESS)
		return BNO055_ERROR;
	if(bus_write(BNO055_MAG_RADIUS_LSB_ADDR, bno055->offset.mag_radius) != BNO055_SUCCESS)
		return BNO055_ERROR;


	return BNO055_SUCCESS;
}

/* Use this function only in config mode */
BNO055_STATUS BNO055_GetCalibrationOffset(bno055_t *bno055){

	bno055->system.opMode = OPERATION_MODE_CONFIG;
	if(BNO055_SetOpMode(bno055) != BNO055_SUCCESS )
		return BNO055_ERROR;

	if(Vector_Read(BNO055_ACCEL_OFFSET_X_LSB_ADDR, bno055->offset.accel_offset, SIX_BYTE) != BNO055_SUCCESS)
		return BNO055_ERROR;

	if(Vector_Read(BNO055_MAG_OFFSET_X_LSB_ADDR, bno055->offset.mag_offset, SIX_BYTE) != BNO055_SUCCESS)
		return BNO055_ERROR;

	if(Vector_Read(BNO055_GYRO_OFFSET_X_LSB_ADDR, bno055->offset.gyro_offset, SIX_BYTE) != BNO055_SUCCESS)
		return BNO055_ERROR;

	if(Vector_Read(BNO055_ACCEL_RADIUS_LSB_ADDR, &bno055->offset.acc_radius, 2) != BNO055_SUCCESS)
		return BNO055_ERROR;

	if(Vector_Read(BNO055_MAG_RADIUS_LSB_ADDR, &bno055->offset.mag_radius, 2) != BNO055_SUCCESS)
		return BNO055_ERROR;

	return BNO055_SUCCESS;
}

BNO055_STATUS BNO055_GetTemperature(bno055_t *bno055){

	if(bus_read(BNO055_TEMP_ADDR, &(bno055->temperature), ONE_BYTE) != BNO055_SUCCESS )
		return BNO055_ERROR;

	return BNO055_SUCCESS;
}

/* for acceleration        -> acc
 * for magnetometer        -> mag
 * for gyroscope           -> gyr
 * for gravity             -> gra
 * for linear acceleration -> lin
 * for euler angles        -> eul
 */
BNO055_STATUS BNO055_Get_Vector(bno055_t *bno055, vector_type vec){

	int16_t data_xyz[BNO055_XYZ_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE};
	if(Vector_Read((uint8_t)vec, data_xyz, SIX_BYTE) != BNO055_SUCCESS )
		return BNO055_ERROR;
	switch (vec){

	case acc:
		/* Read 6 bytes of data; x, y, z axis (two bytes each) */
		bno055->accel.x = ((double)data_xyz[0])/BNO055_ACCEL_DIV_MSQ;
		bno055->accel.y = ((double)data_xyz[1])/BNO055_ACCEL_DIV_MSQ;
		bno055->accel.z = ((double)data_xyz[2])/BNO055_ACCEL_DIV_MSQ;
		break;

	case mag:
		bno055->magn.x = ((double)data_xyz[0])/BNO055_MAG_DIV_UT;
		bno055->magn.y = ((double)data_xyz[1])/BNO055_MAG_DIV_UT;
		bno055->magn.z = ((double)data_xyz[2])/BNO055_MAG_DIV_UT;
		break;

	case gyr:
		bno055->gyro.x = ((double)data_xyz[0])/BNO055_GYRO_DIV_DPS;
		bno055->gyro.y = ((double)data_xyz[1])/BNO055_GYRO_DIV_DPS;
		bno055->gyro.z = ((double)data_xyz[2])/BNO055_GYRO_DIV_DPS;
		break;

	case gra:
		bno055->gravity.x = ((double)data_xyz[0])/BNO055_GRAVITY_DIV_MSQ;
		bno055->gravity.y = ((double)data_xyz[1])/BNO055_GRAVITY_DIV_MSQ;
		bno055->gravity.z = ((double)data_xyz[2])/BNO055_GRAVITY_DIV_MSQ;
		break;

	case lin:
		bno055->linearAccel.x = ((double)data_xyz[0])/BNO055_LINEAR_ACCEL_DIV_MSQ;
		bno055->linearAccel.y = ((double)data_xyz[1])/BNO055_LINEAR_ACCEL_DIV_MSQ;
		bno055->linearAccel.z = ((double)data_xyz[2])/BNO055_LINEAR_ACCEL_DIV_MSQ;
		break;

	case eul:
		bno055->euler.h = ((double)data_xyz[0])/BNO055_EULER_DIV_DEG;
		bno055->euler.r = ((double)data_xyz[1])/BNO055_EULER_DIV_DEG;
		bno055->euler.p = ((double)data_xyz[2])/BNO055_EULER_DIV_DEG;
		break;

	default:
		return BNO055_ERROR;
		break;
	}

	return BNO055_SUCCESS;

}

