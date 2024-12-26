#ifndef BMP280_DRIVER_H
#define BMP280_DRIVER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// BMP280 I2C adresi
#define BMP280_ADDRESS (0x77 << 1)

// BMP280 Register adresleri
#define BMP280_REG_CALIB      0x88
#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7

// Kalibrasyon parametreleri
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t t_fine;
} BMP280_CalibData;

// BMP280 yapılandırma
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    BMP280_CalibData calib;
} BMP280_HandleTypeDef;

// Fonksiyon prototipleri
HAL_StatusTypeDef BMP280_Init(BMP280_HandleTypeDef *bmp, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMP280_ReadID(BMP280_HandleTypeDef *bmp, uint8_t *id);
HAL_StatusTypeDef BMP280_ReadTrim(BMP280_HandleTypeDef *bmp);
HAL_StatusTypeDef BMP280_Reset(BMP280_HandleTypeDef *bmp);
HAL_StatusTypeDef BMP280_Config(BMP280_HandleTypeDef *bmp, uint8_t ctrl_meas, uint8_t config);
HAL_StatusTypeDef BMP280_ReadPressure(BMP280_HandleTypeDef *bmp, float *pressure);

#endif // BMP280_DRIVER_H
