#include "BME280_STM32.h"


// BMP280 register okuma
static HAL_StatusTypeDef BMP280_ReadReg(BMP280_HandleTypeDef *bmp, uint8_t reg, uint8_t *buffer, uint16_t size) {
    return HAL_I2C_Mem_Read(bmp->hi2c, bmp->address, reg, I2C_MEMADD_SIZE_8BIT, buffer, size, HAL_MAX_DELAY);
}

// BMP280 register yazma
static HAL_StatusTypeDef BMP280_WriteReg(BMP280_HandleTypeDef *bmp, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(bmp->hi2c, bmp->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

// BMP280 sensör kimliğini okuma
HAL_StatusTypeDef BMP280_ReadID(BMP280_HandleTypeDef *bmp, uint8_t *id) {
    return BMP280_ReadReg(bmp, BMP280_REG_ID, id, 1);
}

// BMP280 sensör reset
HAL_StatusTypeDef BMP280_Reset(BMP280_HandleTypeDef *bmp) {
    return BMP280_WriteReg(bmp, BMP280_REG_RESET, 0xB6);
}

// BMP280 kalibrasyon verilerini okuma
HAL_StatusTypeDef BMP280_ReadTrim(BMP280_HandleTypeDef *bmp) {
    uint8_t calib[24];
    if (BMP280_ReadReg(bmp, BMP280_REG_CALIB, calib, 24) != HAL_OK) {
        return HAL_ERROR;
    }

    bmp->calib.dig_T1 = (calib[1] << 8) | calib[0];
    bmp->calib.dig_T2 = (calib[3] << 8) | calib[2];
    bmp->calib.dig_T3 = (calib[5] << 8) | calib[4];
    bmp->calib.dig_P1 = (calib[7] << 8) | calib[6];
    bmp->calib.dig_P2 = (calib[9] << 8) | calib[8];
    bmp->calib.dig_P3 = (calib[11] << 8) | calib[10];
    bmp->calib.dig_P4 = (calib[13] << 8) | calib[12];
    bmp->calib.dig_P5 = (calib[15] << 8) | calib[14];
    bmp->calib.dig_P6 = (calib[17] << 8) | calib[16];
    bmp->calib.dig_P7 = (calib[19] << 8) | calib[18];
    bmp->calib.dig_P8 = (calib[21] << 8) | calib[20];
    bmp->calib.dig_P9 = (calib[23] << 8) | calib[22];

    return HAL_OK;
}

// BMP280 yapılandırma
HAL_StatusTypeDef BMP280_Config(BMP280_HandleTypeDef *bmp, uint8_t ctrl_meas, uint8_t config) {
    if (BMP280_WriteReg(bmp, BMP280_REG_CTRL_MEAS, ctrl_meas) != HAL_OK) {
        return HAL_ERROR;
    }
    return BMP280_WriteReg(bmp, BMP280_REG_CONFIG, config);
}

// BMP280 basınç okuma
HAL_StatusTypeDef BMP280_ReadPressure(BMP280_HandleTypeDef *bmp, float *pressure) {
    int32_t raw_press;
    uint8_t data[6];

    if (BMP280_ReadReg(bmp, BMP280_REG_PRESS_MSB, data, 6) != HAL_OK) {
        return HAL_ERROR;
    }

    raw_press = (int32_t)((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;

    // Basınç hesaplama
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp->calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp->calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp->calib.dig_P1) >> 33;

    if (var1 == 0) {
        return HAL_ERROR;
    }

    p = 1048576 - raw_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp->calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->calib.dig_P7) << 4);
    *pressure = (float)p / 256.0f;

    return HAL_OK;
}



// BMP280 başlatma
HAL_StatusTypeDef BMP280_Init(BMP280_HandleTypeDef *bmp, I2C_HandleTypeDef *hi2c) {
    bmp->hi2c = hi2c;
    bmp->address = BMP280_ADDRESS;

    uint8_t id;
    if (BMP280_ReadID(bmp, &id) != HAL_OK || id != 0x58) {
        return HAL_ERROR; // Kimlik doğrulanamadı
    }

    if (BMP280_Reset(bmp) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(100);

    if (BMP280_ReadTrim(bmp) != HAL_OK) {
        return HAL_ERROR;
    }

    return BMP280_Config(bmp, 0x27, 0xA0); // Örnek ölçüm ayarları
}
