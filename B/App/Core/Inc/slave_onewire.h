/*
 * onewire.h
 *
 *  Version 1.0.1
 */

#ifndef SLAVE_ONEWIRE_H_
#define SLAVE_ONEWIRE_H_

#include "stm32f1xx.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
uint8_t command;
bool resetDetected;
uint8_t * dataToSend;
uint16_t sizeToSend;
uint8_t * dtatReceived;
uint16_t sizeToReceive;
bool OverDrive;
bool ResumeCommand;
//TODO: Добавить флаг ошибки
} TllOWdata;

typedef enum {

LLOW_WAIT_RESET,
LLOW_RECEIVE,
LLOW_TRANSMIT

} TllOWCommand;

#endif