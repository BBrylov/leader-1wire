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
#include "onewire.h"
typedef enum
{
  NORMALSPEED = 0,
  OVERDRIVESPEED = 1,
  MAXSPEED
} OverDrive;




// выбираем, на каком USART находится 1-wire
#define OW_UART1
//#define OW_UART2
//#define OW_UART3
//#define OW_UART4

// если нужно отдавать тики FreeRTOS, то раскомментировать
//#define OW_GIVE_TICK_RTOS

// первый параметр функции OW_Send



OverDrive getOverDrive ();
void setOverDrive(OverDrive newOverDrive);

#endif /* ONEWIRE_H_ */
