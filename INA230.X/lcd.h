/*
 * File:   lcd.h
 * Author: lal
 *
 * Created on 16 mars 2015, 13:46
 */

#ifndef _LCD_H
#define _LCD_H

#include "general.h"

#define NB_LINES    2                               // number of display lines
#define NB_COL      16                              // number of characters per line

#define LCD_PWR     LATDbits.LATD7                  // LCD ON/OFF line
#define LCD_EN      LATDbits.LATD6                  // LCD enable line
#define LCD_RW      LATDbits.LATD5                  // LCD read/write line
#define LCD_RS      LATDbits.LATD4                  // LCD register select line
#define LCD_DAT0    LATDbits.LATD0                  // data bit 0
#define LCD_DAT1    LATDbits.LATD1                  // data bit 1
#define LCD_DAT2    LATDbits.LATD2                  // data bit 2
#define LCD_DAT3    LATDbits.LATD3                  // data bit 3

#define LCD_PWR_DIR TRISDbits.TRISD7                // direction bit for the power line
#define LCD_EN_DIR  TRISDbits.TRISD6                // direction for EN control line
#define LCD_RW_DIR  TRISDbits.TRISD5                // direction for RW control line
#define LCD_RS_DIR  TRISDbits.TRISD4                // direction for RS control line
#define LCD_D0_DIR  TRISDbits.TRISD0                // direction for data line 0
#define LCD_D1_DIR  TRISDbits.TRISD1                // direction for data line 1
#define LCD_D2_DIR  TRISDbits.TRISD2                // direction for data line 2
#define LCD_D3_DIR  TRISDbits.TRISD3                // direction for data line 3

#define LCD_ON()    LCD_PWR = ENABLE                // turns the display on
#define LCD_OFF()   LCD_PWR = DISABLE               // turns the display off


void LCDInit(void);                                             // LCD initialization service
void LCDClear(void);                                            // screen clearing service
void LCDGoto(UINT8_T Pos,UINT8_T Ln);                           // cursor positioning service
void LCDPutChar(UINT8_T Data);                                  // outputs ASCII character
void LCDPutByte(UINT8_T Val);                                   // outputs binary value
void LCDWriteStr(const char  *Str);                             // outputs strings to LCD
void LCDWriteHexa(UINT8_T column, UINT8_T row, UINT8_T value);  // output hexadecimal value at specified position
void LCDWriteInt(UINT8_T column, UINT8_T row, UINT8_T value);   // output positive decimal value (from 00 to 99) on 2 digits, at specified position
void LCDWriteFloat(UINT8_T column, UINT8_T row, float value);   // output positive float value as x.yy, at specified position
void LCDShiftDisplay(INT8_T shiftCount);                        // shift display 'shiftCount' on the left

#endif /* _LCD_H */
