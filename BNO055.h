/*
 * BNO055.h
 *
 *  Created on: Feb 4, 2021
 *      Author: Zeynep Yurttaþ
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>

/********************************************************/
/**\name    I2C ADDRESS DEFINITION FOR BNO055           */
/********************************************************/
/* bno055 I2C Address */
#define BNO055_I2C_ADDR1 (0x28)
#define BNO055_I2C_ADDR2 (0x29)

/***************************************************/
/**\name    REGISTER ADDRESS DEFINITION  */
/***************************************************/
/* Page id register definition*/
#define BNO055_PAGE_ID_ADDR                 (0X07)

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                 (0x00)
#define BNO055_ACCEL_REV_ID_ADDR            (0x01)
#define BNO055_MAG_REV_ID_ADDR              (0x02)
#define BNO055_GYRO_REV_ID_ADDR             (0x03)
#define BNO055_SW_REV_ID_LSB_ADDR           (0x04)
#define BNO055_SW_REV_ID_MSB_ADDR           (0x05)
#define BNO055_BL_REV_ID_ADDR               (0X06)

/* Accel data register*/
#define BNO055_ACCEL_DATA_X_LSB_ADDR        (0X08)
#define BNO055_ACCEL_DATA_X_MSB_ADDR        (0X09)
#define BNO055_ACCEL_DATA_Y_LSB_ADDR        (0X0A)
#define BNO055_ACCEL_DATA_Y_MSB_ADDR        (0X0B)
#define BNO055_ACCEL_DATA_Z_LSB_ADDR        (0X0C)
#define BNO055_ACCEL_DATA_Z_MSB_ADDR        (0X0D)

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR          (0X0E)
#define BNO055_MAG_DATA_X_MSB_ADDR          (0X0F)
#define BNO055_MAG_DATA_Y_LSB_ADDR          (0X10)
#define BNO055_MAG_DATA_Y_MSB_ADDR          (0X11)
#define BNO055_MAG_DATA_Z_LSB_ADDR          (0X12)
#define BNO055_MAG_DATA_Z_MSB_ADDR          (0X13)

/*Gyro data registers*/
#define BNO055_GYRO_DATA_X_LSB_ADDR         (0X14)
#define BNO055_GYRO_DATA_X_MSB_ADDR         (0X15)
#define BNO055_GYRO_DATA_Y_LSB_ADDR         (0X16)
#define BNO055_GYRO_DATA_Y_MSB_ADDR         (0X17)
#define BNO055_GYRO_DATA_Z_LSB_ADDR         (0X18)
#define BNO055_GYRO_DATA_Z_MSB_ADDR         (0X19)

/*Euler data registers*/
#define BNO055_EULER_H_LSB_ADDR             (0X1A)
#define BNO055_EULER_H_MSB_ADDR             (0X1B)
#define BNO055_EULER_R_LSB_ADDR             (0X1C)
#define BNO055_EULER_R_MSB_ADDR             (0X1D)
#define BNO055_EULER_P_LSB_ADDR             (0X1E)
#define BNO055_EULER_P_MSB_ADDR             (0X1F)

/*Quaternion data registers*/
#define BNO055_QUATERNION_DATA_W_LSB_ADDR   (0X20)
#define BNO055_QUATERNION_DATA_W_MSB_ADDR   (0X21)
#define BNO055_QUATERNION_DATA_X_LSB_ADDR   (0X22)
#define BNO055_QUATERNION_DATA_X_MSB_ADDR   (0X23)
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR   (0X24)
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR   (0X25)
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR   (0X26)
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR   (0X27)

/* Linear acceleration data registers*/
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR (0X28)
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR (0X29)
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR (0X2A)
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR (0X2B)
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR (0X2C)
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR (0X2D)

/*Gravity data registers*/
#define BNO055_GRAVITY_DATA_X_LSB_ADDR      (0X2E)
#define BNO055_GRAVITY_DATA_X_MSB_ADDR      (0X2F)
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR      (0X30)
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR      (0X31)
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR      (0X32)
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR      (0X33)

/* Temperature data register*/
#define BNO055_TEMP_ADDR                    (0X34)

/* Status registers*/
#define BNO055_CALIB_STAT_ADDR              (0X35)
#define BNO055_SELFTEST_RESULT_ADDR         (0X36)
#define BNO055_INTR_STAT_ADDR               (0X37)
#define BNO055_SYS_CLK_STAT_ADDR            (0X38)
#define BNO055_SYS_STAT_ADDR                (0X39)
#define BNO055_SYS_ERR_ADDR                 (0X3A)

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR                (0X3B)
#define BNO055_DATA_SELECT_ADDR             (0X3C)

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR                (0X3D)
#define BNO055_PWR_MODE_ADDR                (0X3E)

#define BNO055_SYS_TRIGGER_ADDR             (0X3F)
#define BNO055_TEMP_SOURCE_ADDR             (0X40)

/* Axis remap registers*/
#define BNO055_AXIS_MAP_CONFIG_ADDR         (0X41)
#define BNO055_AXIS_MAP_SIGN_ADDR           (0X42)

/* SIC registers*/
#define BNO055_SIC_MATRIX_0_LSB_ADDR        (0X43)
#define BNO055_SIC_MATRIX_0_MSB_ADDR        (0X44)
#define BNO055_SIC_MATRIX_1_LSB_ADDR        (0X45)
#define BNO055_SIC_MATRIX_1_MSB_ADDR        (0X46)
#define BNO055_SIC_MATRIX_2_LSB_ADDR        (0X47)
#define BNO055_SIC_MATRIX_2_MSB_ADDR        (0X48)
#define BNO055_SIC_MATRIX_3_LSB_ADDR        (0X49)
#define BNO055_SIC_MATRIX_3_MSB_ADDR        (0X4A)
#define BNO055_SIC_MATRIX_4_LSB_ADDR        (0X4B)
#define BNO055_SIC_MATRIX_4_MSB_ADDR        (0X4C)
#define BNO055_SIC_MATRIX_5_LSB_ADDR        (0X4D)
#define BNO055_SIC_MATRIX_5_MSB_ADDR        (0X4E)
#define BNO055_SIC_MATRIX_6_LSB_ADDR        (0X4F)
#define BNO055_SIC_MATRIX_6_MSB_ADDR        (0X50)
#define BNO055_SIC_MATRIX_7_LSB_ADDR        (0X51)
#define BNO055_SIC_MATRIX_7_MSB_ADDR        (0X52)
#define BNO055_SIC_MATRIX_8_LSB_ADDR        (0X53)
#define BNO055_SIC_MATRIX_8_MSB_ADDR        (0X54)

/* Accelerometer Offset registers*/
#define BNO055_ACCEL_OFFSET_X_LSB_ADDR      (0X55)
#define BNO055_ACCEL_OFFSET_X_MSB_ADDR      (0X56)
#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR      (0X57)
#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR      (0X58)
#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR      (0X59)
#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR      (0X5A)

/* Magnetometer Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB_ADDR        (0X5B)
#define BNO055_MAG_OFFSET_X_MSB_ADDR        (0X5C)
#define BNO055_MAG_OFFSET_Y_LSB_ADDR        (0X5D)
#define BNO055_MAG_OFFSET_Y_MSB_ADDR        (0X5E)
#define BNO055_MAG_OFFSET_Z_LSB_ADDR        (0X5F)
#define BNO055_MAG_OFFSET_Z_MSB_ADDR        (0X60)

/* Gyroscope Offset registers*/
#define BNO055_GYRO_OFFSET_X_LSB_ADDR       (0X61)
#define BNO055_GYRO_OFFSET_X_MSB_ADDR       (0X62)
#define BNO055_GYRO_OFFSET_Y_LSB_ADDR       (0X63)
#define BNO055_GYRO_OFFSET_Y_MSB_ADDR       (0X64)
#define BNO055_GYRO_OFFSET_Z_LSB_ADDR       (0X65)
#define BNO055_GYRO_OFFSET_Z_MSB_ADDR       (0X66)

/* Radius registers*/
#define BNO055_ACCEL_RADIUS_LSB_ADDR        (0X67)
#define BNO055_ACCEL_RADIUS_MSB_ADDR        (0X68)
#define BNO055_MAG_RADIUS_LSB_ADDR          (0X69)
#define BNO055_MAG_RADIUS_MSB_ADDR          (0X6A)

/* PAGE0 REGISTERS DEFINITION END*/
/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define BNO055_ACCEL_CONFIG_ADDR            (0X08)
#define BNO055_MAG_CONFIG_ADDR              (0X09)
#define BNO055_GYRO_CONFIG_ADDR             (0X0A)
#define BNO055_GYRO_MODE_CONFIG_ADDR        (0X0B)
#define BNO055_ACCEL_SLEEP_CONFIG_ADDR      (0X0C)
#define BNO055_GYRO_SLEEP_CONFIG_ADDR       (0X0D)
#define BNO055_MAG_SLEEP_CONFIG_ADDR        (0x0E)

/* Interrupt registers*/
#define BNO055_INT_MASK_ADDR                (0X0F)
#define BNO055_INT_ADDR                     (0X10)
#define BNO055_ACCEL_ANY_MOTION_THRES_ADDR  (0X11)
#define BNO055_ACCEL_INTR_SETTINGS_ADDR     (0X12)
#define BNO055_ACCEL_HIGH_G_DURN_ADDR       (0X13)
#define BNO055_ACCEL_HIGH_G_THRES_ADDR      (0X14)
#define BNO055_ACCEL_NO_MOTION_THRES_ADDR   (0X15)
#define BNO055_ACCEL_NO_MOTION_SET_ADDR     (0X16)
#define BNO055_GYRO_INTR_SETING_ADDR        (0X17)
#define BNO055_GYRO_HIGHRATE_X_SET_ADDR     (0X18)
#define BNO055_GYRO_DURN_X_ADDR             (0X19)
#define BNO055_GYRO_HIGHRATE_Y_SET_ADDR     (0X1A)
#define BNO055_GYRO_DURN_Y_ADDR             (0X1B)
#define BNO055_GYRO_HIGHRATE_Z_SET_ADDR     (0X1C)
#define BNO055_GYRO_DURN_Z_ADDR             (0X1D)
#define BNO055_GYRO_ANY_MOTION_THRES_ADDR   (0X1E)
#define BNO055_GYRO_ANY_MOTION_SET_ADDR     (0X1F)

/* PAGE1 REGISTERS DEFINITION END*/

/***************************************************/
/**\name    CONSTANT DEFINITIONS                   */
/***************************************************/
#define BNO055_INIT_VALUE                          ((uint8_t)0)
#define BNO055_TIMEOUT								100

/* Page ID */
#define BNO055_PAGE_ZERO                           (0X00)
#define BNO055_PAGE_ONE                            (0X01)

#define ONE_BYTE								   ((uint8_t) 1)
#define SIX_BYTE								   ((uint8_t) 6)

#define RST_SYS_ENABLE    						    (0X20)
#define POWER_MODE_NORMAL							(0)

#define BNO055_CHIP_ID                             (0xA0)

/*Accel division factor*/
#define BNO055_ACCEL_DIV_MSQ                       (100.0)

/*Mag division factor*/
#define BNO055_MAG_DIV_UT                          (16.0)

/*Gyro division factor*/
#define BNO055_GYRO_DIV_DPS                        (16.0)

/*Gravity accel division factor*/
#define BNO055_GRAVITY_DIV_MSQ                     (100.0)

/*Linear accel division factor*/
#define BNO055_LINEAR_ACCEL_DIV_MSQ                (100.0)

/*Euler division factor*/
#define BNO055_EULER_DIV_DEG                       (16.0)
/****************************************************/
/**\name    ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define BNO055_XYZ_DATA_SIZE    		           (3)
/**************************************************************/
/**\name    STRUCTURE DEFINITIONS                         */
/**************************************************************/
typedef enum {
	BNO055_SUCCESS = 0,
	BNO055_ERROR = -1,
	BNO055_E_NULL_PTR = -127,
} BNO055_STATUS;

typedef struct{
	double x;
	double y;
	double z;
}vector;

typedef struct{
	double h;
	double r;
	double p;
}eulervector;

typedef enum {
	OPERATION_MODE_CONFIG,
	OPERATION_MODE_ACCONLY,
	OPERATION_MODE_MAGONLY,
	OPERATION_MODE_GYRONLY,
	OPERATION_MODE_ACCMAG,
	OPERATION_MODE_ACCGYRO,
	OPERATION_MODE_MAGGYRO,
	OPERATION_MODE_AMG,
	OPERATION_MODE_IMUPLUS,
	OPERATION_MODE_COMPASS,
	OPERATION_MODE_M4G,
	OPERATION_MODE_NDOF_FMC_OFF,
	OPERATION_MODE_NDOF,
} t_OperationMode;

typedef struct {
	uint8_t data;
	uint8_t sys;
	uint8_t accel;
	uint8_t gyro;
	uint8_t mag;
} BNO_Calib;

typedef struct {
	uint8_t page_id;
	uint8_t chip_id;
	uint8_t dev_addr; /**< i2c device address of bno055 */
	t_OperationMode opMode;
	uint8_t stResult; /*self-test result*/
	uint8_t sysStatus;
	uint8_t sysError;
}BNO_System;

typedef struct {
	int16_t accel_offset[3];
	int16_t mag_offset[3];
	int16_t gyro_offset[3];
	int16_t acc_radius;
	int16_t mag_radius;

}offset;
typedef struct {

	BNO_Calib calibration;
	BNO_System system;
	vector accel;
	vector magn;
	vector gyro;
	vector gravity;
	vector linearAccel;
	eulervector euler;
	uint8_t temperature;
	offset offset;
}bno055_t;

typedef enum {
	acc = BNO055_ACCEL_DATA_X_LSB_ADDR,
	mag = BNO055_MAG_DATA_X_LSB_ADDR,
	gyr = BNO055_GYRO_DATA_X_LSB_ADDR,
	gra = BNO055_GRAVITY_DATA_X_LSB_ADDR,
	lin = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
	eul = BNO055_EULER_H_LSB_ADDR,
}vector_type;


/*************************************************/
/**\name FUNCTION DECLARATION    */
/*************************************************/
BNO055_STATUS BNO055_Init(bno055_t *bno055);
BNO055_STATUS BNO055_BuildInSelfTest(bno055_t *bno055);
BNO055_STATUS BNO055_SystemStatus(bno055_t *bno055);
BNO055_STATUS BNO055_CalibrationStatus(bno055_t *bno055);
BNO055_STATUS BNO055_GetTemperature(bno055_t *bno055);
BNO055_STATUS BNO055_Get_Vector(bno055_t *bno055, vector_type vec);
BNO055_STATUS BNO055_SetOpMode(bno055_t *bno055);
BNO055_STATUS BNO055_SetCalibrationOffset(bno055_t *bno055);
BNO055_STATUS BNO055_GetCalibrationOffset(bno055_t *bno055);

#endif /* INC_BNO055_H_ */
