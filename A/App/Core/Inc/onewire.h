/*
 * onewire.h
 *
 *  Version 1.0.1
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_


// для разных процессоров потребуется проверить функцию OW_Init
// на предмет расположения ножек USART
#include "stm32f1xx.h"

// выбираем, на каком USART находится 1-wire
#define OW_UART1
//#define OW_UART2
//#define OW_UART3
//#define OW_UART4

// если нужно отдавать тики FreeRTOS, то раскомментировать
//#define OW_GIVE_TICK_RTOS

// первый параметр функции OW_Send
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

uint8_t OW_Init();
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart);
uint8_t OW_Reset(int8_t OverDrive);
int8_t getOverDrive ();
void setOverDrive(int8_t newOverDrive);

#endif /* ONEWIRE_H_ */
