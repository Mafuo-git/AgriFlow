/* 
 * File:   uart.h
 * Author: lal
 *
 * Created on 9 avril 2015, 07:42
 */

#ifndef _UART_H
#define	_UART_H

#include "general.h"


void UARTInit(UINT16_T baudRate);           // init UART with specified baud rate
UINT8_T UARTReadByte(void);                 // read a byte from UART
void UARTWriteByte(UINT8_T data);           // write a byte to UART
void UARTWriteStr(char *Str);               // output strings to UART
void UARTWriteStrLn(char *Str);             // output strings to UART
void UARTWriteByteHex(UINT8_T data);        // write the hexadecimal value of a bit in a readable manner
void UARTWriteStrHex(char *data);
void UARTWriteByteDec(UINT8_T data);

#endif	/* _UART_H */

