/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */

#include "onewire.h"
#include "usart.h"

// Буфер для приема/передачи по 1-wire
uint8_t owRxBuffer[8];
uint8_t owTxBuffer[8];
static TowData owData ={
    .command=UARTNONE,
    .OwerDrive=0,
    .rxBuffer=owRxBuffer,
    .rxSize=sizeof(owRxBuffer),
    .txBuffer=owTxBuffer,
    .txSize=sizeof(owTxBuffer),
};


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
int8_t OW_Init() {
	uartInit();
 	return HAL_OK;
 }

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
int8_t OW_Reset() {
	uint8_t ow_presence = 0xf0;
    uint8_t ow_return=0xf0;

    owData.command=UARTRESET;
    owData.OwerDrive=0;
    owData.txBuffer=&ow_presence;
    owData.txSize=sizeof(ow_presence);
    owData.rxBuffer=&ow_return;
    owData.rxSize=sizeof(ow_return);

    uartSendReset(&owData);

	if (ow_presence != ow_return) {
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
int8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen,
		uint8_t *data, uint8_t dLen, uint8_t readStart) {

	// если требуется сброс - сбрасываем и проверяем на наличие устройств
	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset() == OW_NO_DEVICE) {
			return OW_NO_DEVICE;
		}
	}

    owData.command=UARTSEND;
    owData.OwerDrive=0;
    owData.rxBuffer=owRxBuffer;
    owData.rxSize=sizeof(owRxBuffer);
    owData.txBuffer=owTxBuffer;
    owData.txSize=sizeof(owTxBuffer);

	while (cLen > 0) {

		OW_toBits(*command, owTxBuffer);
		command++;
		cLen--;



        uartSendReceive(&owData);

// 		DMA_InitTypeDef DMA_InitStructure;

// 		// DMA на чтение
// 		DMA_DeInit(OW_DMA_CH_RX);
// 		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DR);
// 		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
// 		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
// 		DMA_InitStructure.DMA_BufferSize = 8;
// 		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// 		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
// 		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
// 		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
// 		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
// 		DMA_Init(OW_DMA_CH_RX, &DMA_InitStructure);

// 		// DMA на запись
// 		DMA_DeInit(OW_DMA_CH_TX);
// 		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DR);
// 		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
// 		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
// 		DMA_InitStructure.DMA_BufferSize = 8;
// 		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// 		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
// 		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
// 		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
// 		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
// 		DMA_Init(OW_DMA_CH_TX, &DMA_InitStructure);

// 		// старт цикла отправки
// 		USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
// 		USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
// 		DMA_Cmd(OW_DMA_CH_RX, ENABLE);
// 		DMA_Cmd(OW_DMA_CH_TX, ENABLE);

// 		// Ждем, пока не примем 8 байт
// 		while (DMA_GetFlagStatus(OW_DMA_FLAG) == RESET){
// #ifdef OW_GIVE_TICK_RTOS
// 			taskYIELD();
// #endif
// 		}

// 		// отключаем DMA
// 		DMA_Cmd(OW_DMA_CH_TX, DISABLE);
// 		DMA_Cmd(OW_DMA_CH_RX, DISABLE);
// 		USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);

		// если прочитанные данные кому-то нужны - выкинем их в буфер
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte(owTxBuffer);
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


    //-----------------------------------------------------------------------------
    // Данная функция осуществляет сканирование сети 1-wire и записывает найденные
    //   ID устройств в массив buf, по 8 байт на каждое устройство.
    // переменная num ограничивает количество находимых устройств, чтобы не переполнить
    // буфер.
    //-----------------------------------------------------------------------------
    static int8_t OW_Scan(uint8_t * buf, uint8_t num) {

        uint8_t found = 0;
        uint8_t * lastDevice;
        uint8_t * curDevice = buf;
        uint8_t numBit, lastCollision, currentCollision, currentSelection;

        lastCollision = 0;
        while (found < num) {

            currentCollision = 0;
            // посылаем команду на поиск устройств

            if (OW_Send(1,(uint8_t *)"\xf0",1, NULL,0, OW_NO_READ) < 0)
                return OW_NO_DEVICE;

            for (numBit = 1; numBit <= 64; numBit++) {
                // читаем два бита. Основной и комплементарный
                OW_toBits(0xff, owTxBuffer);

                if (SendData(2) < 0)
                    return OW_ERROR_WIRE;

                if (owRxBuffer[0] == 0xff) {   // читаем два бита. Основной и комплементарный
                    if (owRxBuffer[1] == 0xff) {
                        // две единицы, где-то провтыкали и заканчиваем поиск
                        return found;
                    } else {
                        // 10 - на данном этапе только 1
                        currentSelection = 1;
                    }
                } else {
                    if (owRxBuffer[1] == 0xff) {
                        // 01 - на данном этапе только 0
                        currentSelection = 0;
                    } else {
                        // 00 - коллизия
                        if (numBit < lastCollision) {
                            // идем по дереву, не дошли до развилки
                            if (lastDevice[(numBit - 1) >> 3] & 1 << ((numBit - 1) & 0x07)) {
                                // (numBit-1)>>3 - номер байта
                                // (numBit-1)&0x07 - номер бита в байте
                                currentSelection = 1;

                                // если пошли по правой ветке, запоминаем номер бита
                                if (currentCollision < numBit) {
                                    currentCollision = numBit;
                                }
                            } else {
                                currentSelection = 0;
                            }
                        } else {
                            if (numBit == lastCollision) {
                                currentSelection = 0;
                            } else {
                                // идем по правой ветке
                                currentSelection = 1;

                                // если пошли по правой ветке, запоминаем номер бита
                                if (currentCollision < numBit) {
                                    currentCollision = numBit;
                                }
                            }
                        }
                    }
                }

                if (currentSelection == 1) {
                    curDevice[(numBit - 1) >> 3] |= 1 << ((numBit - 1) & 0x07);
                    // (numBit-1)>>3 - номер байта
                    // (numBit-1)&0x07 - номер бита в байте
                    OW_toBits(0x01, owTxBuffer);
                } else {
                    curDevice[(numBit - 1) >> 3] &= ~(1 << ((numBit - 1) & 0x07));
                    // (numBit-1)>>3 - номер байта
                    // (numBit-1)&0x07 - номер бита в байте
                    OW_toBits(0x00, owTxBuffer);
                }
                if (SendData(1) < 0)
                    return OW_ERROR_WIRE;
            }
            found++;
            lastDevice = curDevice;
            curDevice += 8;
            if (currentCollision == 0)
                return found;

            lastCollision = currentCollision;
        }

        return found;
    }