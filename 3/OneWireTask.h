/*
 * OneWireTask.h
 *
 *  Created on: 20 дек. 2020 г.
 *      Author: Brylov Boris
 */

#ifndef ONEWIRETASK_H_
#define ONEWIRETASK_H_

int8_t OneWireTaskStart(void);           //Запускатель задачи OneWire
void OneWireTask(void * pvParametres);   //Задача по считыванию датчиков  OneWire


#endif /* ONEWIRETASK_H_ */
