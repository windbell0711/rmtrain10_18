
#ifndef RM10_18_BMI_H
#define RM10_18_BMI_H

#include "main.h"
#include "spi.h"

void bmi088_write_byte(uint8_t tx_data);

void bmi088_read_byte(uint8_t *rx_data, uint8_t length);

void bmi088_write_reg(uint8_t reg, uint8_t data);

#endif //RM10_18_BMI_H
