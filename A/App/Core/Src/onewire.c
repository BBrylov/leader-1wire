/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */

#include "onewire.h"
#include "stm32f1xx_hal.h"

//*************************************************
#ifdef OW_UART1

#undef OW_UART2
#undef OW_UART3
#undef OW_UART4

#define OW_UART 		USART1
#define OW_DMA_CH_RX 	DMA1_Channel5
#define OW_DMA_CH_TX 	DMA1_Channel4
#define OW_DMA_FLAG		DMA_FLAG_TC5

#endif

#ifdef OW_UART2

#undef OW_UART1
#undef OW_UART3
#undef OW_UART4

#define OW_UART 		UART2
#define OW_DMA_CH_RX 	DMA1_Channel6
#define OW_DMA_CH_TX 	DMA1_Channel7
#define OW_DMA_FLAG		DMA_FLAG_TC6

#endif
//**************************************************************

#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel4
#define USARTx_RX_DMA_CHANNEL             DMA1_Channel5


/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Channel4_IRQn
#define USARTx_DMA_RX_IRQn                DMA1_Channel5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Channel4_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA1_Channel5_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(ow_buf) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

static OverDrive=0;
void setOverDrive(int8_t newOverDrive){
    if (newOverDrive!=0)
        OverDrive=1;
    else OverDrive=0;
}

int8_t getOverDrive (){
    return OverDrive;
}

// Буфер для приема/передачи по 1-wire
uint8_t ow_buf[8];

#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff

//-----------------------------------------------------------------------------
// функция преобразует один байт в восемь, для передачи через USART
// ow_byte - байт, который надо преобразовать
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
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
uint8_t OW_toByte(uint8_t *ow_bits) {
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
	// GPIO_InitTypeDef GPIO_InitStruct;
	// UART_InitTypeDef UART_InitStructure;

	// if (OW_UART == UART1) {
	// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
	// 			ENABLE);

	// 	// USART TX
	// 	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	// 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	// 	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	// 	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 	// USART RX
	// 	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	// 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	// 	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	// 	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);

	// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// }

	// if (OW_UART == UART2) {
	// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
	// 			ENABLE);

	// 	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	// 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	// 	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	// 	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	// 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	// 	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	// 	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// }

	// UART_InitStructure.USART_BaudRate = 115200;
	// UART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// UART_InitStructure.USART_StopBits = USART_StopBits_1;
	// UART_InitStructure.USART_Parity = USART_Parity_No;
	// UART_InitStructure.USART_HardwareFlowControl =
	// 		USART_HardwareFlowControl_None;
	// UART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	// UART_Init(OW_UART, &UART_InitStructure);
	// UART_Cmd(OW_UART, ENABLE);
	return OW_OK;
}

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
uint8_t OW_Reset(int8_t OverDrive) {
	uint8_t ow_presence;
	USART_HandleTypeDef UartHandle;

    UartHandle.Instance        = USARTx;
    UartHandle.Init.BaudRate   = 9600;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        Error_Handler();
    }
    while (HAL_USART_GetState(&UartHandle)==HAL_USART_STATE_READY);

    HAL_UART_Transmit(&UartHandle, (uint8_t *)"\xf0", 1, UART_SEND_TIMEOUT);
    while (LL_USART_IsActiveFlag_TC(OW_UART)==RESET);

    HAL_UART_Receive(&UartHandle, &ow_presence, 1, UART_RECEIVE_TIMEOUT);

    UartHandle.Instance        = USARTx;
    UartHandle.Init.BaudRate   = 115200;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        Error_Handler();
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

		DMA_HandleTypeDef DMA_InitStructure;

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

        DMA_InitStructure.Instance = OW_DMA_CH_RX;
        DMA_InitStructure.Init.Direction = DMA_PERIPH_TO_MEMORY;
        DMA_InitStructure.Init.PeriphInc = DMA_PINC_DISABLE;
        DMA_InitStructure.Init.MemInc = DMA_MINC_ENABLE;
        DMA_InitStructure.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        DMA_InitStructure.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        DMA_InitStructure.Init.Mode = DMA_NORMAL;
        DMA_InitStructure.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&DMA_InitStructure) != HAL_OK)
        {
          //Error_Handler();
        }


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
