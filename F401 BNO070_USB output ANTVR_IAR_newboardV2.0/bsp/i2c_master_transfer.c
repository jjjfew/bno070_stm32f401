/*
 * Since STM32CubeMX doesn't support a combined I2C write/read
 * transfer, we implement one here. E.g., there's no I2C stop
 * between the write and the read parts of the transfer. The
 * BNO070 requires these transfers for many operations.
 */

#include "stm32f4xx_hal.h"

/**
  * @brief  This function handles I2C Communication Timeout.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Flag: specifies the I2C flag to check.
  * @param  Status: The new Flag status (SET or RESET).
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout)
{
  uint32_t timeout = 0;

  timeout = HAL_GetTick() + Timeout;

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_I2C_GET_FLAG(hi2c, Flag) == RESET)
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if(HAL_GetTick() >= timeout)
        {
          hi2c->State= HAL_I2C_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hi2c);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_I2C_GET_FLAG(hi2c, Flag) != RESET)
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if(HAL_GetTick() >= timeout)
        {
          hi2c->State= HAL_I2C_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hi2c);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for Master addressing phase.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Flag: specifies the I2C flag to check.
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnMasterAddressFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, uint32_t Timeout)
{
  uint32_t timeout = 0;

  timeout = HAL_GetTick() + Timeout;

  while(__HAL_I2C_GET_FLAG(hi2c, Flag) == RESET)
  {
    if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
    {
      /* Generate Stop */
      hi2c->Instance->CR1 |= I2C_CR1_STOP;

      /* Clear AF Flag */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

      hi2c->ErrorCode = HAL_I2C_ERROR_AF;
      hi2c->State= HAL_I2C_STATE_READY;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if(Timeout != HAL_MAX_DELAY)
    {
      if(HAL_GetTick() >= timeout)
      {
        hi2c->State= HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

/**
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress: Target device address
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_MasterRequestWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout)
{
  /* Generate Start */
  hi2c->Instance->CR1 |= I2C_CR1_START;         //产生开始信号

  /* Wait until SB flag is set */
  if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  if(hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
  {
    /* Send slave address */
    hi2c->Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(DevAddress);    //发送7位地址
  }

  /* Wait until ADDR flag is set */
  if(I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout) != HAL_OK)   //等待地址设置正确
  {
    if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
    {
      return HAL_ERROR;
    }
    else
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/**
  * @brief  Master sends target device address for read request.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress: Target device address
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_MasterRequestRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout)
{
  /* Enable Acknowledge */
  hi2c->Instance->CR1 |= I2C_CR1_ACK;

  /* Generate Start */
  hi2c->Instance->CR1 |= I2C_CR1_START;

  /* Wait until SB flag is set */
  if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  if(hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
  {
    /* Send slave address */
    hi2c->Instance->DR = __HAL_I2C_7BIT_ADD_READ(DevAddress);
  }

  /* Wait until ADDR flag is set */
  if(I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout) != HAL_OK)
  {
    if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
    {
      return HAL_ERROR;
    }
    else
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress: Target device address
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Transfer(I2C_HandleTypeDef *hi2c, 
                                          uint16_t DevAddress, 
                                          uint8_t *txData, 
                                          uint16_t txSize, 
                                          uint8_t *rxData, 
                                          uint16_t rxSize, 
                                          uint32_t Timeout)
{
  if(hi2c->State != HAL_I2C_STATE_READY)
    return HAL_BUSY;

  if (txSize == 0 && rxSize == 0)
    return HAL_ERROR;
  
  if(txSize != 0 && txData == NULL)
    return  HAL_ERROR;

  if(rxSize != 0 && rxData == NULL)
    return  HAL_ERROR;

  if (rxSize == 0)
    return HAL_I2C_Master_Transmit(hi2c, DevAddress, txData, txSize, Timeout);  //write data
  if (txSize == 0)
    return HAL_I2C_Master_Receive(hi2c, DevAddress, rxData, rxSize, Timeout);   //read data 

  if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)           //既有读又有写时
  { 
    printf("HAL_BUSY 1...\n"); 
    return HAL_BUSY;
  }
  
  /* Process Locked */
  __HAL_LOCK(hi2c);  // LOCK hi2c
  
 if (txSize > 0) 
  { 
    hi2c->State = HAL_I2C_STATE_BUSY_TX;   //指示处于发送状态
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
   
    /* Send Slave Address */
    if(I2C_MasterRequestWrite(hi2c, DevAddress, Timeout) != HAL_OK)        //先发送write address
    {
      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        return HAL_TIMEOUT;
      }
    }

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

    while(txSize > 0)
    {
      /* Wait until TXE flag is set */
      if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TXE, RESET, Timeout) != HAL_OK)
      {
        __HAL_UNLOCK(hi2c);
        return HAL_TIMEOUT;
      }

      /* Write data to DR */
      hi2c->Instance->DR = (*txData++);                          //write data
      txSize--;

      if((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET) && (txSize != 0))
      {
        /* Write data to DR */
        hi2c->Instance->DR = (*txData++);
        txSize--;
      }
    }

    /* Wait until TXE flag is set */
    if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TXE, RESET, Timeout) != HAL_OK)
    {
        __HAL_UNLOCK(hi2c);
      return HAL_TIMEOUT;
    }

    /* If not receiving anything, then stop */
    if (rxSize == 0)
    {
      hi2c->Instance->CR1 |= I2C_CR1_STOP;

      /* Wait until BUSY flag is reset */
      if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, Timeout) != HAL_OK)
      {
          __HAL_UNLOCK(hi2c);
          return HAL_TIMEOUT;
      }
      
        hi2c->State = HAL_I2C_STATE_READY;
      __HAL_UNLOCK(hi2c);
      return HAL_OK;
    }
  }
  
  hi2c->State = HAL_I2C_STATE_BUSY_RX;
  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

  /* Send Slave Address */
  if(I2C_MasterRequestRead(hi2c, DevAddress, Timeout) != HAL_OK)       //再发送slave address
  {
    if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }
    else
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_TIMEOUT;
    }
  }
  
  if(rxSize == 1)
  {
    /* Disable Acknowledge */
    hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

    /* Generate Stop */
    hi2c->Instance->CR1 |= I2C_CR1_STOP;
  }
  else if(rxSize == 2)
  {
    /* Disable Acknowledge */
    hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

    /* Enable Pos */
    hi2c->Instance->CR1 |= I2C_CR1_POS;

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
  }
  else
  {
    /* Enable Acknowledge */
    hi2c->Instance->CR1 |= I2C_CR1_ACK;

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
  }

  uint8_t *lenByte = rxData; /* Only look at first byte for length since only support packets much less than 255 bytes */
  int handledLength = 0;
  while(rxSize > 0)
  {
    if(rxSize <= 3)
    {
      /* One byte */
      if(rxSize == 1)
      {
        /* Wait until RXNE flag is set */
        if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, RESET, Timeout) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        /* Read data from DR */
        (*rxData++) = hi2c->Instance->DR;
        rxSize--;
      }
      /* Two bytes */
      else if(rxSize == 2)
      {
        /* Wait until BTF flag is set */
        if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        /* Generate Stop */
        hi2c->Instance->CR1 |= I2C_CR1_STOP;

        /* Read data from DR */
        (*rxData++) = hi2c->Instance->DR;
        rxSize--;

        /* Read data from DR */
        (*rxData++) = hi2c->Instance->DR;
        rxSize--;
      }
      /* 3 Last bytes */
      else
      {
        /* Wait until BTF flag is set */
        if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        /* Disable Acknowledge */
        hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

        /* Read data from DR */
        (*rxData++) = hi2c->Instance->DR;
        rxSize--;

        /* Wait until BTF flag is set */
        if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        /* Generate Stop */
        hi2c->Instance->CR1 |= I2C_CR1_STOP;

        /* Read data from DR */
        (*rxData++) = hi2c->Instance->DR;
        rxSize--;

        /* Read data from DR */
        (*rxData++) = hi2c->Instance->DR;
        rxSize--;
      }
    }
    else
    {
      /* Wait until RXNE flag is set */
      if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, RESET, Timeout) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }

      /* Read data from DR */
      (*rxData++) = hi2c->Instance->DR;
     
      if (!handledLength) 
      {
        handledLength = 1;
        int specifiedSize = *lenByte;
        if (specifiedSize < rxSize)
        {
          rxSize = specifiedSize;
          if (rxSize < 4)
            rxSize = 4;
        }
      }
      rxSize--;
      if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET)
      {
        /* Read data from DR */
        (*rxData++) = hi2c->Instance->DR;
        rxSize--;
      }
    }
  }

  /* Disable Pos */
  hi2c->Instance->CR1 &= ~I2C_CR1_POS;

  /* Wait until BUSY flag is reset */
  if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  hi2c->State = HAL_I2C_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}
