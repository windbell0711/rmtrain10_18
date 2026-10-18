#include "bmi.h"

// 参考
// HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout); // blocking mode, IT/DMA better
// HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

// 尝试完成
void bmi088_write_byte(uint8_t tx_data) {
  HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
}

void bmi088_read_byte(uint8_t *rx_data, uint8_t length) {
  HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
}

void bmi088_write_reg(uint8_t reg, uint8_t data) {
  bmi088_write_byte(0x7F & reg);
  bmi088_write_byte(data);
} // 向寄存器中写入数据，可参考5.3，5.4寄存器写入方法，先写首位判别符+地址，再写data
