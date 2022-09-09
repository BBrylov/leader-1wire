/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
static UART_HandleTypeDef huart1 = {0};
static UART_HandleTypeDef huart2 = {0};
static DMA_HandleTypeDef hdma_usart1_rx = {0};
static DMA_HandleTypeDef hdma_usart1_tx = {0};
static DMA_HandleTypeDef hdma_usart2_rx = {0};
static DMA_HandleTypeDef hdma_usart2_tx = {0};



void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void DMA1_Channel6_IRQHandler(void);
void DMA1_Channel7_IRQHandler(void);

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle);

__IO UartStatus UartReady = ITStatusRESET;

void owInit(){

    huart1.Instance=USART1;
    HAL_UART_MspInit(&huart1);

}

/* USER CODE END 0 */


/* USART1 init function */

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
      if(uartHandle->Instance==USART1)
      {
      /* USER CODE BEGIN USART1_MspInit 0 */

      /* USER CODE END USART1_MspInit 0 */
        /* USART1 clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 DMA Init */
        /* USART1_RX Init */
        hdma_usart1_rx.Instance = DMA1_Channel5;
        hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_rx.Init.Mode = DMA_NORMAL;
        hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
        {
          //break;
        }

        __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

        /* USART1_TX Init */
        hdma_usart1_tx.Instance = DMA1_Channel4;
        hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_tx.Init.Mode = DMA_NORMAL;
        hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
        {
          //break;
        }

        __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

      /* USER CODE BEGIN USART1_MspInit 1 */

      /* USER CODE END USART1_MspInit 1 */
      }
      else if(uartHandle->Instance==USART2)
      {
      /* USER CODE BEGIN USART2_MspInit 0 */

      /* USER CODE END USART2_MspInit 0 */
        /* USART2 clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART2 DMA Init */
        /* USART2_RX Init */
        hdma_usart2_rx.Instance = DMA1_Channel6;
        hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode = DMA_NORMAL;
        hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
        {
          //break;
        }

        __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

        /* USART2_TX Init */
        hdma_usart2_tx.Instance = DMA1_Channel7;
        hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_tx.Init.Mode = DMA_NORMAL;
        hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
        {
          //break;
        }

        __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

      /* USER CODE BEGIN USART2_MspInit 1 */

      /* USER CODE END USART2_MspInit 1 */
      }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}



// Буфер для приема/передачи по 1-wire
static  uint8_t ow_buf[8];

#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff

#define OW_SEND_RESET		1
#define OW_NO_RESET		2

#define UART_SEND_TIMEOUT 100       //Время в милисекундах
#define UART_RECEIVE_TIMEOUT 100    //Время в милисекундах
// статус возврата функций
#define OW_OK			1
#define OW_ERROR		2
#define OW_NO_DEVICE	3

#define OW_NO_READ		0xff

#define OW_READ_SLOT	0xff
//-----------------------------------------------------------------------------
// функция преобразует один байт в восемь, для передачи через USART
// ow_byte - байт, который надо преобразовать
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
static void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
static uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}

//-----------------------------------------------------------------------------
// инициализирует USART и DMA
//-----------------------------------------------------------------------------
uint8_t OW_Init() {
    owInit();
	return OW_OK;
}



//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
uint8_t OW_Reset(OverDrive overDrive) {
	uint8_t ow_presence;

    UartHandle.Instance        = USARTx;
    UartHandle.Init.BaudRate   = getOverDriveResetSpeed(getOverDrive());
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;

    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
    {
        //Error_Handler();
    }

    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        //Error_Handler();
    }

    while (HAL_UART_GetState(&UartHandle)==HAL_UART_STATE_READY);

    HAL_UART_Transmit(&UartHandle, (uint8_t *)"\xf0", 1, UART_SEND_TIMEOUT);

    //while ( LL_UART_IsActiveFlag_TC(USARTx) == RESET );

    HAL_UART_Receive(&UartHandle, &ow_presence, 1, UART_RECEIVE_TIMEOUT);

    UartHandle.Instance        = USARTx;
    UartHandle.Init.BaudRate   = getOverDriveSpeed(getOverDrive());
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
    {
        //Error_Handler();
    }
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        //Error_Handler();
    }

	if (ow_presence != 0xf0) {
		return OW_OK;
	}

	return OW_NO_DEVICE;
}

//-----------------------------------------------------------------------------
// процедура общения с шиной 1-wire
// sendReset - посылать RESET в начале общения.
// 		OW_SEND_RESET или OW_NO_RESET
// command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOTH
// cLen - длина буфера команд, столько байт отошлется в шину
// data - если требуется чтение, то ссылка на буфер для чтения
// dLen - длина буфера для чтения. Прочитается не более этой длины
// readStart - с какого символа передачи начинать чтение (нумеруются с 0)
//		можно указать OW_NO_READ, тогда можно не задавать data и dLen
//-----------------------------------------------------------------------------
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen,
		uint8_t *data, uint8_t dLen, uint8_t readStart) {


	// если требуется сброс - сбрасываем и проверяем на наличие устройств
	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset(1) == OW_NO_DEVICE) {
			return OW_NO_DEVICE;
		}
	}

	while (cLen > 0) {

		OW_toBits(*command, ow_buf);
		command++;
		cLen--;

        owSendRecieve(&owOutputBuffer,  &owInputBuffet, sizeof(owOutputBuffer))

		// DMA на чтение
		// HAL_DMA_DeInit(OW_DMA_CH_RX);
		// DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DR);
		// DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
		// DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		// DMA_InitStructure.DMA_BufferSize = 8;
		// DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		// DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		// DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		// DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		// DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		// DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		// DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		// DMA_Init(OW_DMA_CH_RX, &DMA_InitStructure);

		// DMA на запись
		// DMA_DeInit(OW_DMA_CH_TX);
		// DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DR);
		// DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
		// DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		// DMA_InitStructure.DMA_BufferSize = 8;
		// DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		// DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		// DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		// DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		// DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		// DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		// DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		// DMA_Init(OW_DMA_CH_TX, &DMA_InitStructure);

    /* USART1 DMA Init */
    /* USART1_RX Init */

		// старт цикла отправки
		// USART_ClearFlag(OW_UART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
		// USART_DMACmd(OW_UART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
		// DMA_Cmd(OW_DMA_CH_RX, ENABLE);
		// DMA_Cmd(OW_DMA_CH_TX, ENABLE);

		// Ждем, пока не примем 8 байт
		//while (DMA_GetFlagStatus(OW_DMA_FLAG) == RESET)
            {
#ifdef OW_GIVE_TICK_RTOS
			taskYIELD();
#endif
		    }

		// отключаем DMA
		// DMA_Cmd(OW_DMA_CH_TX, DISABLE);
		// DMA_Cmd(OW_DMA_CH_RX, DISABLE);
		// USART_DMACmd(OW_UART, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);

		// если прочитанные данные кому-то нужны - выкинем их в буфер
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		} else {
			if (readStart != OW_NO_READ) {
				readStart--;
			}
		}
	}

	return OW_OK;
}
