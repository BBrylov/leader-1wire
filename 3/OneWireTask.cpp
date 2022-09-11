/*
 * OneWireTask.cpp
 *
 *  Created on: 20 дек. 2020 г.
 *      Author: Brylov Boris
 */
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include "task_defs.h"
#include "queue.h"

#include <stdio.h>
#include "stddef.h"
#include "stdint.h"
#include <stdio.h>
#include <string.h>


#include "ModBusAddress.h"
#include "ModbusData.h"
#include "OneWire.h"
#include "OneWireTask.h"
#include "IntSensors.h"
#define TEMP_BUF_LEN 16


namespace OneWire {
typedef struct {
    char address[8];
} TDeviceAddress;

typedef enum {
    ReadRom = 0xbe,
    SkipRom = 0xcc,
    MatchRom = 0x55,
    ConvertTemp = 0x44,

    WriteRom = 0x4e
} TCommand;

typedef struct __attribute__((packed)) {
    int16_t Tempreture;
    uint8_t Th;
    uint8_t Tl;
    uint8_t Config;
} TRom;

typedef struct __attribute__((packed)) {
    char RomCommand;
    TDeviceAddress DeviceAddress;
    char Command;
} TOutBuffer;
#warning Fixmi Create union on one buffer for both objects

class TOneWireDevice {
  public:
    TOneWireDevice(TDeviceAddress * DeviceAddress);
    int8_t MeasureAll();
    int8_t Measure();
    int16_t ReadTemp();

  protected:
    TDeviceAddress DeviceAddress;
    const int8_t AddresSize = 8;
    int8_t ErrorDevice = 0;
    int16_t Temperature;
};

TDeviceAddress Address[2];
TOneWireDevice * FirstDS1820;
TOneWireDevice * SecondDS1820;

TOneWireDevice::TOneWireDevice(TDeviceAddress * pDeviceAddress) {
    memcpy((void *)&DeviceAddress, (void *)pDeviceAddress, AddresSize);
}

int8_t TOneWireDevice::MeasureAll() {
    return Mcucpp::OneWire1::OW_Send((uint8_t *)"\xcc\x44", 2, NULL, 0, OW_NO_READ);
}

int8_t TOneWireDevice::Measure() {
    int8_t ret = 0;
    char Buffer[TEMP_BUF_LEN];
    memset(Buffer, 0xff, sizeof(Buffer));
    TOutBuffer * OutBufer = (TOutBuffer *)Buffer;
    OutBufer->RomCommand = MatchRom;
    OutBufer->Command = ConvertTemp;
    memcpy((void *)&OutBufer->DeviceAddress, (void *)&DeviceAddress, AddresSize);
    ret = Mcucpp::OneWire1::OW_Send((uint8_t *)Buffer,
                                    1              // match rom command
                                            + 8    // AddresDS
                                            + 1,   // Command readrom
                                    0,
                                    0,             //читаем 0
                                    OW_NO_READ);   //начинаем читать после 1+8+1 байт
    if (ret < 0)
        ErrorDevice--;
    return ret;
}

int16_t TOneWireDevice::ReadTemp() {
    int8_t ret = 0;
    char Buffer[TEMP_BUF_LEN];
    memset(Buffer, 0xff, sizeof(Buffer));
    TOutBuffer * OutBufer = (TOutBuffer *)Buffer;
    memcpy((void *)&OutBufer->DeviceAddress, (void *)&DeviceAddress, AddresSize);
    OutBufer->RomCommand = MatchRom;
    OutBufer->Command = ReadRom;
    ret = Mcucpp::OneWire1::OW_Send((uint8_t *)Buffer,
                                    1              // match rom command
                                            + 8    // AddresDS
                                            + 1    // Command readrom
                                            + 6,   //читаем первые 2 байта
									(uint8_t *)Buffer,        //читаем сюда
                                    6,             //читаем 2 байта
                                    10);           //начинаем читать после 1+8+1 байт
    if (ret > 0) {
        Temperature = Buffer[1] << 8 | Buffer[0];
        ;
        return Temperature;
    } else
        return ret;
}


void OneWireTask(void * pvParametres) {   //Задача по считыванию датчиков  OneWire
    int8_t found1WireDevice = 0;
    Mcucpp::OneWire1::Init();
    found1WireDevice = Mcucpp::OneWire1::OW_Scan((uint8_t *)Address, 2);
    FirstDS1820 = NULL;
    SecondDS1820 = NULL;
    TISensMess ISensMess;

    printf("\Found %d 1Wire  device\n", found1WireDevice);
    if (found1WireDevice > 0) {
        while (1) {
            switch (found1WireDevice) {
            case 2:
                SecondDS1820 = new TOneWireDevice(&Address[1]);
            case 1:
                FirstDS1820 = new TOneWireDevice(&Address[0]);
                break;
            default:
#warning "fixmi out to log "
                printf("Wrong device count %d\n", found1WireDevice);
                break;
            }
            for (;;) {
                FirstDS1820->MeasureAll();
                vTaskDelay(1000);
                {
                    int32_t t1,t2 = 0;
                    if (FirstDS1820) {
                        t1 = FirstDS1820->ReadTemp() * 625 / 1000;
                        //*(fModbus(8448))=(float)t1/10;
                    }
                    if (SecondDS1820) {
                        t2 = SecondDS1820->ReadTemp() * 625 / 1000;
                        //*(fModbus(8450))=(float)t2/10;
                    }
                	ISensMess.TypeOfMess	= SetModbus;
          			ISensMess.ModbusAddress	= HVS_TEMPERATURE;
          			ISensMess.TypeValue = single;
          			ISensMess.FValue = (float)t1/10;
           			xQueueSendToBack(QHandleISens,&ISensMess,QuieryWaitTime);

                    //printf("T1=%d, T2=%d\n", t1, t2);
                }
            }
        }
    } else
        printf("\n1Wire  device not found\n");
    printf("Task OneWire End\n");
    vTaskDelete(NULL);
};

}   // namespace OneWire
int8_t OneWireTaskStart(void) {   //Запускатель задачи OneWire
   return xTaskCreate(OneWire::OneWireTask, "OneWireTask", ONEWIRETASK_SIZE, NULL, configSTANDART_LEVEL_INT, NULL);   //&xDefaultTaskHandle);
}
