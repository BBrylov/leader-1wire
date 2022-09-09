/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */

#include "onewire.h"
#include "stm32f1xx_hal.h"
#include "usart.h"

//*************************************************
/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(ow_buf) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

static OverDrive overDrive=0;

void setOverDrive(OverDrive newOverDrive){
    if (newOverDrive!=0)
        overDrive=1;
    else overDrive=0;
}


OverDrive getOverDrive (){
    return overDrive;
}

static uint32_t getOverDriveSpeed (OverDrive overDrive){
        uint32_t result=115200;
            switch (overDrive)
            {
                case OVERDRIVESPEED: result=921600;
                    break;
                default:
                    break;
            }
    return result;
}

static uint32_t getOverDriveResetSpeed (OverDrive overDrive){
        uint32_t result=9600;
            switch (overDrive)
            {
                case OVERDRIVESPEED: result=76800;
                    break;
                default:
                    break;
            }
    return result;
}


