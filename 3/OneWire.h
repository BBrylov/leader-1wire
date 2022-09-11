/*
 * OneWire.h
 *
 *  Created on: 5 нояб. 2020 г.
 *      Author: Brylov Boris
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_
#include <clock.h>
#include <iopins.h>
#include <pinlist.h>

//#include <platform_dalay.h>
#include "OneWireDefs.h"
#include <BitSet.h>
#include <delay.h>

#ifdef USE_FREERTOS
#include <FreeRTOS.h>

#endif

#define BUF_LEN 8
#define OW_NO_READ 0xff

namespace Mcucpp {
namespace Private {
typedef IO::PinList< IO::Pa9 > Usart1Pins;

typedef IO::PinList< IO::Pa2 > Usart2Pins;

typedef IO::PinList< IO::Pb10 > Usart3Pins;

typedef IO::PinList< IO::Pc10 > Usart4Pins;

typedef IO::PinList< IO::Pc12 > Usart5Pins;
IO_STRUCT_WRAPPER(USART1, USART1_REGS, USART_TypeDef);
IO_STRUCT_WRAPPER(USART2, USART2_REGS, USART_TypeDef);
IO_STRUCT_WRAPPER(USART3, USART3_REGS, USART_TypeDef);
IO_STRUCT_WRAPPER(UART4, USART4_REGS, USART_TypeDef);
IO_STRUCT_WRAPPER(UART5, USART5_REGS, USART_TypeDef);
IO_STRUCT_WRAPPER(DMA1, DMA1_REGS, DMA_TypeDef);
IO_STRUCT_WRAPPER(DMA2, DMA2_REGS, DMA_TypeDef);
IO_STRUCT_WRAPPER(DMA1_Channel4, DMA1Channel4_REGS, DMA_Channel_TypeDef);
IO_STRUCT_WRAPPER(DMA1_Channel5, DMA1Channel5_REGS, DMA_Channel_TypeDef);
IO_STRUCT_WRAPPER(DMA1_Channel6, DMA1Channel6_REGS, DMA_Channel_TypeDef);
IO_STRUCT_WRAPPER(DMA1_Channel7, DMA1Channel7_REGS, DMA_Channel_TypeDef);

}   // namespace Private

static volatile uint8_t ow_buf[BUF_LEN] = {0};   // Буфер для приема/передачи по 1-wire
template< class UsartRegs, class DMAReg, class ClockCtrl, class DmaChannelRegRX, class DmaChannelRegTX, class InputPins, uint32_t DMAFlag >
class OWBase {
  public:
    typedef ClockCtrl Clock;
    typedef InputPins Pins;
    static const uint16_t Brr9600 = 0x1d4c;
    static const uint16_t Brr115200 = 0x271;


    static void OW_toBits(uint8_t ow_byte,volatile uint8_t * ow_bits) {   // функция преобразует один байт в восемь, для передачи через USART
        // ow_byte - байт, который надо преобразовать
        // ow_bits - ссылка на буфер, размером не менее 8 байт
        uint8_t i;
        for (i = 0; i < 8; i++) {
            if (ow_byte & 0x01) {
                *ow_bits = 0xff;   // OW_1;
            } else {
                *ow_bits = 0;   // OW_0;
            }
            ow_bits++;
            ow_byte = ow_byte >> 1;
        }
    }
    static  uint8_t OW_toByte(uint8_t * ow_bits) {   // обратное преобразование - из того, что получено через USART опять собирается байт
        // ow_bits - ссылка на буфер, размером не менее 8 байт
        uint8_t ow_byte, i;
        ow_byte = 0;
        for (i = 0; i < 8; i++) {
            ow_byte = ow_byte >> 1;
            if (*ow_bits == 0xff) {   // OW_R_1) {
                ow_byte |= 0x80;
            }
            ow_bits++;
        }

        return ow_byte;
    }
    static int8_t SendData(uint8_t NumBytes) {
        DmaChannelRegTX()->CCR &= ~(1 << (MaskToBit(DMA_CCR_EN)));
        DmaChannelRegRX()->CCR &= ~(1 << (MaskToBit(DMA_CCR_EN)));
        //Далее, инициализация регистров адреса памяти, периферии и количества передаваемых данных:
        DmaChannelRegRX()->CPAR = (uint32_t) & (UsartRegs()->RDR);      //заносим адрес регистра  в CPAR
        DmaChannelRegRX()->CCR = 0 << (MaskToBit(DMA_CCR_MEM2MEM))      //режим MEM2MEM отключен
                                 | 0x00 << (MaskToBit(DMA_CCR_PL))      //приоритет низкий
                                 | 0x00 << (MaskToBit(DMA_CCR_MSIZE))   //разрядность данных в памяти 00-8bit 10-32 bit
                                 | 0x00 << (MaskToBit(DMA_CCR_PSIZE))   //разрядность регистра данных 00-8bit 10-32 bit
                                 | 1 << (MaskToBit(DMA_CCR_MINC))       //Инкремент адреса памяти
                                 | 0 << (MaskToBit(DMA_CCR_PINC))       //Инкремент адреса периферии
                                 | 0 << (MaskToBit(DMA_CCR_CIRC))       //кольцевой режим
                                 | 0 << (MaskToBit(DMA_CCR_DIR))        // 1 - из памяти в периферию
                                 | 0 << (MaskToBit(DMA_CCR_TCIE));
        DmaChannelRegRX()->CMAR = (uint32_t)ow_buf;   //заносим адрес данных в регистр CMAR
        DmaChannelRegRX()->CNDTR = NumBytes;          //количество передаваемых данных


        //Далее, инициализация регистров адреса памяти, периферии и количества передаваемых данных:
        DmaChannelRegTX()->CPAR = (uint32_t) & (UsartRegs()->TDR);      //заносим адрес регистра в CPAR
        DmaChannelRegTX()->CCR = 0 << (MaskToBit(DMA_CCR_MEM2MEM))      //режим MEM2MEM отключен
                                 | 0x00 << (MaskToBit(DMA_CCR_PL))      //приоритет низкий
                                 | 0x00 << (MaskToBit(DMA_CCR_MSIZE))   //разрядность данных в памяти 00-8bit 10-32 bit
                                 | 0x00 << (MaskToBit(DMA_CCR_PSIZE))   //разрядность регистра данных 00-8bit 10-32 bit
                                 | 1 << (MaskToBit(DMA_CCR_MINC))       //Инкремент адреса памяти
                                 | 0 << (MaskToBit(DMA_CCR_PINC))       //Инкремент адреса периферии
                                 | 0 << (MaskToBit(DMA_CCR_CIRC))       //кольцевой режим
                                 | 1 << (MaskToBit(DMA_CCR_DIR))        // 1 - из памяти в периферию
                                 | 0 << (MaskToBit(DMA_CCR_TCIE));
        DmaChannelRegTX()->CMAR = (uint32_t)(ow_buf);   //заносим адрес данных в регистр CMAR
        DmaChannelRegTX()->CNDTR = NumBytes;            //количество передаваемых данных
        DmaChannelRegRX()->CCR |= (1 << (MaskToBit(DMA_CCR_EN)));
        DmaChannelRegTX()->CCR |= (1 << (MaskToBit(DMA_CCR_EN)));
        uint32_t timeoutCounter = 0;
        // Ждем окончания передачи
        while ((!VerifyTC()) && (timeoutCounter < TimeoutTransfer)) {
            timeoutCounter++;
            Wait();
        }
        if (VerifyTC()) {
            ClearTC();
            return Success;
        } else {
            return Error1Wire;
        }
    }

    static void Init() {   // инициализирует USART и DMA
        Clock::Enable();
        Mcucpp::Clock::Dma1Clock::Enable();
        Mcucpp::Clock::Dma2Clock::Enable();
        Pins::SetConfiguration(Pins::AltFunc);
        Pins::SetSpeed(Pins::Speed::Fastest);
        Pins::SetDriverType(Pins::OpenDrain);
        Pins::AltFuncNumber(7);
    }
    static void SetSpeed(const uint16_t Div) {
        DmaChannelRegRX()->CCR &= ~(1 << (MaskToBit(DMA_CCR_EN)));
        DmaChannelRegTX()->CCR &= ~(1 << (MaskToBit(DMA_CCR_EN)));
        UsartRegs()->CR1 = 0;
        UsartRegs()->CR3 = 0 << (MaskToBit(USART_CR3_WUFIE))        // Wakeup from Stop mode interrupt enable
                           | 0 << (MaskToBit(USART_CR3_WUS))        // Wakeup from Stop mode interrupt flag selection
                           | 0 << (MaskToBit(USART_CR3_SCARCNT))    // Smartcard auto-retry count
                           | 0 << (MaskToBit(USART_CR3_DEP))        // Driver enable polarity selection
                           | 0 << (MaskToBit(USART_CR3_DEM))        // Driver enable mode
                           | 0 << (MaskToBit(USART_CR3_DDRE))       // DMA Disable on Reception Error
                           | 0 << (MaskToBit(USART_CR3_OVRDIS))     // Overrun Disable
                           | 0 << (MaskToBit(USART_CR3_ONEBIT))     // One sample bit method enable
                           | 0 << (MaskToBit(USART_CR3_CTSIE))      // CTS interrupt enable
                           | 0 << (MaskToBit(USART_CR3_CTSE))       // CTS enable
                           | 0 << (MaskToBit(USART_CR3_RTSE))       // RTS enable
                           | 1 << (MaskToBit(USART_CR3_DMAT))       // DMA enable transmitter
                           | 1 << (MaskToBit(USART_CR3_DMAR))       // DMA enable receiver
                           | 0 << (MaskToBit(USART_CR3_SCEN))       // Smartcard mode enable
                           | 0 << (MaskToBit(USART_CR3_NACK))       // Smartcard NACK enable
                           | 1 << (MaskToBit(USART_CR3_HDSEL))      // Half-duplex selection
                           | 0 << (MaskToBit(USART_CR3_IRLP))       // IrDA low-power
                           | 0 << (MaskToBit(USART_CR3_IREN))       // IrDA mode enable
                           | 0 << (MaskToBit(USART_CR3_EIE));       // Error interrupt enable
        UsartRegs()->CR2 = 0 << (MaskToBit(USART_CR2_ADD))          // Address of the USART node
                           | 0 << (MaskToBit(USART_CR2_RTOEN))      // Receiver timeout enable
                           | 0 << (MaskToBit(USART_CR2_ABRMODE))    // Auto baud rate mode
                           | 0 << (MaskToBit(USART_CR2_ABREN))      // Auto baud rate enable
                           | 0 << (MaskToBit(USART_CR2_MSBFIRST))   // Most significant bit first
                           | 0 << (MaskToBit(USART_CR2_DATAINV))    // Binary data inversion
                           | 0 << (MaskToBit(USART_CR2_TXINV))      // TX pin active level inversion
                           | 0 << (MaskToBit(USART_CR2_RXINV))      // RX pin active level inversion
                           | 0 << (MaskToBit(USART_CR2_SWAP))       // Swap TX/RX pins
                           | 0 << (MaskToBit(USART_CR2_LINEN))      // LIN mode enable
                           | 0 << (MaskToBit(USART_CR2_STOP))       // STOP bits
                           | 0 << (MaskToBit(USART_CR2_CLKEN))      // Clock pin enable
                           | 0 << (MaskToBit(USART_CR2_CPOL))       // Clock pin polarity
                           | 0 << (MaskToBit(USART_CR2_CPHA))       // Clock phase
                           | 0 << (MaskToBit(USART_CR2_LBCL))       // Last bit clock pulse
                           | 0 << (MaskToBit(USART_CR2_LBDIE))      // LIN break detection interrupt enable
                           | 0 << (MaskToBit(USART_CR2_LBDL))       // LIN break detection lenght
                           | 0 << (MaskToBit(USART_CR2_ADDM7));     // 7-bit Address Detection/4-bit Address Detection
        UsartRegs()->CR1 =
                0 << (MaskToBit(USART_CR1_M))          // Word lenght
                | 0 << (MaskToBit(USART_CR1_EOBIE))    // End of Block interrupt enable
                | 0 << (MaskToBit(USART_CR1_RTOIE))    // Receiver timeout interrupt enable
                | 0 << (MaskToBit(USART_CR1_DEAT))     // If the Driver Enable feature is not supported, this bit is reserved and must be kept
                | 0 << (MaskToBit(USART_CR1_DEDT))     // If the Driver Enable feature is not supported, this bit is reserved and must be kept
                | 0 << (MaskToBit(USART_CR1_OVER8))    // Oversampling mode
                | 0 << (MaskToBit(USART_CR1_CMIE))     // Character match interrupt enable
                | 0 << (MaskToBit(USART_CR1_MME))      // Mute mode enable
                | 0 << (MaskToBit(USART_CR1_M0))       // Word lenght
                | 0 << (MaskToBit(USART_CR1_WAKE))     // Receiver wakeup method
                | 0 << (MaskToBit(USART_CR1_PCE))      // Parity control
                | 0 << (MaskToBit(USART_CR1_PS))       // Parity control selection
                | 0 << (MaskToBit(USART_CR1_PEIE))     // PE interrupt enable
                | 0 << (MaskToBit(USART_CR1_TXEIE))    // interrupt enable
                | 0 << (MaskToBit(USART_CR1_TCIE))     // Transmission complete interrupt enable
                | 0 << (MaskToBit(USART_CR1_RXNEIE))   // RXNE interrupt enable
                | 0 << (MaskToBit(USART_CR1_IDLEIE))   // IDLE interrupt enable
                | 1 << (MaskToBit(USART_CR1_TE))       // Transmitter enable
                | 1 << (MaskToBit(USART_CR1_RE))       // Receiver enable
                | 0 << (MaskToBit(USART_CR1_UESM))     // USART enable in Stop mode
                | 1 << (MaskToBit(USART_CR1_UE));      // USART enable


        UsartRegs()->BRR = Div;
        UsartRegs()->ICR |= USART_ICR_TCCF;
    }
    static void Wait() {
#ifdef USE_FREERTOS
        vTaskDelay(1);   // если есть OS отдаем ей время
#else
        delay_ms< 1, 72000000 >;
#endif
    }
    static void StopData() {
        DmaChannelRegRX()->CCR &= ~(1 << (MaskToBit(DMA_CCR_EN)));
        DmaChannelRegTX()->CCR &= ~(1 << (MaskToBit(DMA_CCR_EN)));
    }
    static bool VerifyTC(void) { return ((DMAReg()->ISR & DMAFlag) != 0); }
    static void ClearTC(void) { DMAReg()->IFCR |= DMAFlag; }

    static int8_t OwSendReset() {
        SetSpeed(Brr9600);   //скорость для передачи сброса
        ow_buf[0] = 0xf0;    // Байт для формирования импульса
        if (SendData(1) < 0)
            return Error1Wire;   // Передаем и принимаем в буфер ow_buf один байт

        SetSpeed(Brr115200);
        if (ow_buf[0] < 0xff)
            return Success;
        else
            return NotDetectReset;
    }

    //-----------------------------------------------------------------------------
    // процедура общения с шиной 1-wire
    // sendReset - посылать RESET в начале общения.
    // 		OW_SEND_RESET или OW_NO_RESET
    // Command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOT
    // Len - длина буфера команд, столько байт отошлется в шину
    // ReadData - если требуется чтение, то ссылка на буфер для чтения
    // ReadLen - длина буфера для чтения. Прочитается не более этой длины
    // StartBitRead - с какого символа передачи начинать чтение (нумеруются с 0)
    //		можно указать OW_NO_READ, тогда можно не задавать data и dLen
    //-----------------------------------------------------------------------------
    static int8_t OW_Send(uint8_t * Command, uint8_t Len,  uint8_t * ReadData, uint8_t ReadLen, uint8_t StartBitRead) {
        if (OwSendReset() < 0) {   // сбрасываем и проверяем на наличие устройств
            return Not1WireDevises;
        }
        while (Len > 0) {
            OW_toBits(*Command, ow_buf);   // преобразуем байт в битобайты для передачи
            Command++;
            Len--;
            if (SendData(8) < 0)
                return Error1Wire;   // посылаем битобайты
            if (ReadData && (StartBitRead == 0) && ReadLen) {
                *ReadData = OW_toByte(ow_buf);
                ReadData++;
                ReadLen--;
            } else if (StartBitRead != OW_NO_READ) {
                StartBitRead--;
            }
        }
        return Success;
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

            if (OW_Send((uint8_t *)"\xf0", 1, 0, 0, OW_NO_READ) < 0)
                return Not1WireDevises;

            for (numBit = 1; numBit <= 64; numBit++) {
                // читаем два бита. Основной и комплементарный
                OW_toBits(0xff, ow_buf);

                if (SendData(2) < 0)
                    return Error1Wire;

                if (ow_buf[0] == 0xff) {   // читаем два бита. Основной и комплементарный
                    if (ow_buf[1] == 0xff) {
                        // две единицы, где-то провтыкали и заканчиваем поиск
                        return found;
                    } else {
                        // 10 - на данном этапе только 1
                        currentSelection = 1;
                    }
                } else {
                    if (ow_buf[1] == 0xff) {
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
                    OW_toBits(0x01, ow_buf);
                } else {
                    curDevice[(numBit - 1) >> 3] &= ~(1 << ((numBit - 1) & 0x07));
                    // (numBit-1)>>3 - номер байта
                    // (numBit-1)&0x07 - номер бита в байте
                    OW_toBits(0x00, ow_buf);
                }
                if (SendData(1) < 0)
                    return Error1Wire;
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
};


typedef OWBase< Private::USART1_REGS, Private::DMA1_REGS, Mcucpp::Clock::Usart1Clock, Private::DMA1Channel5_REGS, Private::DMA1Channel4_REGS,
                Private::Usart1Pins, DMA_ISR_TCIF5 >
        OneWire1;
};     // namespace Mcucpp
#endif /* ONEWIRE_H_ */
