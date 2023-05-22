#include "adxl345.h"

static HAL_StatusTypeDef adxl_write(adxl_Config_t *adxl_cfg, uint8_t reg, uint8_t val);



int8_t adxl345_init(adxl_Config_t *adxl_cfg) {
	uint8_t chip_id;
	if (HAL_I2C_Mem_Read (adxl_cfg->hi2c, ADXL_I2C_RADDR, ADXL_REG_DEVID, 1, &chip_id, 1, 100) != HAL_OK) {
		return -2;
	}
	else if (chip_id != ADXL_CHIP_ID) {
		return -1;
	}

	// data_format range= +- 4g
	if (adxl_write(adxl_cfg, ADXL_REG_DATA_FORMAT, ADXL_DFORMAT_4G) != HAL_OK) {
		return -2;
	}
	// Reset all bits
	if (adxl_write(adxl_cfg, ADXL_REG_POWER_CTL, ADXL_RESET) != HAL_OK) {
		return -2;
	}
	if (adxl_write(adxl_cfg, ADXL_REG_POWER_CTL, (1 << ADXL_POWER_CTRL_MEASURE)) != HAL_OK) {
		return -2;  // power_cntl measure and wake up 8hz
	}
	return 0;
}

int8_t adxl345_get_xyz(adxl_Config_t *adxl_cfg, float *RxBuffer) {
	// 6 bytes: contains x, y, z
	uint8_t RxBuffer_int[6];
	memset(RxBuffer_int, 0, sizeof(RxBuffer_int));
	if (HAL_I2C_Mem_Read (adxl_cfg->hi2c, ADXL_I2C_RADDR, ADXL_REG_DATAX0, 1, RxBuffer_int, ADXL_DATA_READ, 100) != HAL_OK) {
		return -2;
	}

	RxBuffer[0] = ((float)((int16_t )((RxBuffer_int[1]<<8)|RxBuffer_int[0])))*0.0078;
	RxBuffer[1] = ((float)((int16_t )((RxBuffer_int[3]<<8)|RxBuffer_int[2])))*0.0078;
	RxBuffer[2] = ((float)((int16_t )((RxBuffer_int[5]<<8)|RxBuffer_int[4])))*0.0078;

	return 0;
}


HAL_StatusTypeDef adxl_write(adxl_Config_t *adxl_cfg, uint8_t reg, uint8_t val) {
	uint8_t data[2];
	data[0] = reg;
	data[1] = val;
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit (adxl_cfg->hi2c, ADXL_I2C_RADDR, data, 2, 100);
	return ret;
}
