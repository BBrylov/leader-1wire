/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
typedef enum
{
  ITStatusRESET = 0,
  ITStatusSET = !ITStatusRESET
}  UartStatus;

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
typedef enum{
    UARTSEND,
    UARTRESEIVE,
    UARTRESET,
    UARTNONE,
} Tcommand;

typedef struct {
    uint8_t OwerDrive;
    uint8_t *txBuffer;
    uint8_t txSize;
    uint8_t *rxBuffer;
    uint8_t rxSize;
    Tcommand command;
} TowData;

void uartInit();
int8_t uartSendReset  (TowData *owData);
int8_t uartSendReceive(TowData *owData);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
