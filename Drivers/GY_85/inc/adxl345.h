#ifndef ADXL345_H_
#define ADXL345_H_
#include "stm32f4xx_hal.h"

typedef struct {
	I2C_HandleTypeDef* hi2c;
} adxl_Config_t;


/***************************************************
 * Device Parameters
 ***************************************************/
// 1: write, 0: read
#define ADXL_I2C_ADDR					0x53U
#define ADXL_I2C_RADDR					(ADXL_I2C_ADDR << 1)
#define ADXL_I2C_WADDR					((ADXL_I2C_ADDR << 1)+0x01)
/***************************************************
 * Register map
 ***************************************************/

#define ADXL_REG_DEVID					0x00U
#define ADXL_REG_THRESH_TAP				0x1DU
#define ADXL_REG_OFSX					0x1EU
#define ADXL_REG_OFSY					0x1FU
#define ADXL_REG_OFSZ					0x20U
#define ADXL_REG_DUR					0x21U
#define ADXL_REG_LATENT					0x22U
#define ADXL_REG_WINDOW					0x23U
#define ADXL_REG_THRESH_ACT				0x24U
#define ADXL_REG_THRESH_INACT			0x25U
#define ADXL_REG_TIME_INACT				0x26U
#define ADXL_REG_TACT_INACT_CTL			0x27U

#define ADXL_REG_POWER_CTL				0x2dU

#define ADXL_REG_DATA_FORMAT			0x31U
#define ADXL_REG_DATAX0					0x32U
#define ADXL_REG_DATAX1					0x33U
#define ADXL_REG_DATAY0					0x34U
#define ADXL_REG_DATAY1					0x35U
#define ADXL_REG_DATAZ0					0x36U
#define ADXL_REG_DATAZ1					0x37U

/***************************************************
 * Methods
 ***************************************************/
/// DATA FORMAT CONFIGURATIONS
#define ADXL_DFORMAT_2G					0
#define ADXL_DFORMAT_4G					1
#define ADXL_DFORMAT_8G					2
#define ADXL_DFORMAT_16G				3

/***************************************************
 * Bit positions
 ***************************************************/

//// POWER_CTRL BIT
// WAKEUP_BIT
#define ADXL_POWER_CTRL_WAKEUP			0

// MEASURE
#define ADXL_POWER_CTRL_MEASURE			3


/***************************************************
 * Random Values
 ***************************************************/

#define ADXL_CHIP_ID					0xE5

#define ADXL_DATA_READ					6

#define ADXL_SET						1
#define ADXL_RESET						0

/// ERROR CODES
#define ADXL_I2C_ERR					-2
#define ADXL_VAL_ERR					-1
#define ADXL_OK							0

/***************************************************
 * Methods
 ***************************************************/


// Init / Deinit
int8_t adxl345_init(adxl_Config_t *adxl_cfg);
int8_t adxl345_deinit();

// Read
int8_t adxl345_get_xyz(adxl_Config_t *adxl_cfg, float *RxBuffer);

#endif
