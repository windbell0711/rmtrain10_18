
#ifndef RM10_18_ACCEL_GYRO_H
#define RM10_18_ACCEL_GYRO_H

#include "main.h"

void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);

void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);

void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);

void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data);

uint8_t bmi088_init(void);

#endif //RM10_18_ACCEL_GYRO_H
