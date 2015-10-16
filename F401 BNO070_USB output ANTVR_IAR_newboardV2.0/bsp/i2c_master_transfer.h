#ifndef I2C_MASTER_TRANSFER_H
#define I2C_MASTER_TRANSFER_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef HAL_I2C_Master_Transfer(I2C_HandleTypeDef *hi2c, 
                                          uint16_t DevAddress, 
                                          uint8_t *txData, 
                                          uint16_t txSize, 
                                          uint8_t *rxData, 
                                          uint16_t rxSize, 
                                          uint32_t Timeout);

#endif /* I2C_MASTER_TRANSFER_H */