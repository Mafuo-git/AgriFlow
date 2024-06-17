/* 
 * File:   mesure_temp.c
 * Author: Jean-Marc Capron
 *
 * Created on 29 avril 2015, 16:36
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// PIC18F46K22 Configuration Bit Settings

#include <xc.h>
#include "lcd.h"
#include "general.h"
#include "i2c.h"

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1H
#pragma config FOSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, CLKOUT function on OSC2)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

void I2C_Write_Register(unsigned char reg, unsigned int value) {
    i2c_start();
    i2c_write((TC74_ADDRESS << 1) | 0); // Send address with write condition
    i2c_write(reg); // Send register address
    i2c_write((value >> 8) & 0xFF); // Send high byte of value
    i2c_write(value & 0xFF); // Send low byte of value
    i2c_stop();
}

bool I2C_Write(uint8_t data) {
    SSPBUF = data; // Write data to SSPBUF
    while(SSPSTATbits.BF); // Wait until write cycle is complete
    while(SSPCON2bits.ACKSTAT); // Wait for ACK/NACK from slave
    return !SSPCON2bits.ACKSTAT; // Return ACK/NACK status
}

void I2C_Scan(void) {
    uint8_t address;
    for(address = 1; address < 128; address++) {
        i2c_start();
        if(I2C_Write(address << 1)) { // Shift address and add R/W bit (0 for write)
            // If ACK received, print address
            LCDWriteHexa(7, 0, address);
        }
        i2c_stop();
        __delay_ms(10); // Small delay between each address probe
    }
}

int main(int argc, char** argv) {

    INT8_T temp, temp2;                             // temperature (1 byte)
    char string[16];
    char degre[2] = {0xDF, '\0'};            // ASCII code for ° symbol, not known by XC8 compiler, see HD44780 datasheet

    LCDInit();
    LCDClear();

    i2c_init();                             // configuration de l'interface I2C
    I2C_Write_Register(0x05, 0x0697); // calibration du current
     _delay(12500);

    LCDGoto(0, 0);                          // write fixed text to avoid blinking
    strcpy(string, "Watt =   ");
    LCDWriteStr(string);
    
    /*while(1) {
        I2C_Scan();
        __delay_ms(5000); // Wait 5 seconds before scanning again
    }
    
    while (1) {
        
    }*/

    forever {

        temp = 0;
        temp2 = 0;

        LCDGoto(0, 1);
        LCDWriteStr("Conversion");
        _delay(125000);                         // wait for 125000 Tcy = 125000 * 4us = 0.5 s

        i2c_start();                                    // send start condition
        i2c_write((TC74_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x00) ;                               // select current register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((TC74_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
        temp = i2c_read();
        i2c_ACK();
        temp2 = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();                                     // send stop condition
        
        i2c_start();
//        temp = abs(i2c_read());                         // read temperature (only 0 to 99 degrés Celsius)
        i2c_write((TC74_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x09) ;                               // select current register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((TC74_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
        
        INT8_T check_mesure = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();
        
        while (check_mesure != 0){
            _delay(5);
        }
        
        i2c_start();
        i2c_write((TC74_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x00) ;                               // select current register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((TC74_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
        
        temp = i2c_read();
        i2c_ACK();
        temp2 = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();                                     // send stop condition

        LCDWriteHexa(7, 0, temp);
        LCDWriteHexa(10, 0, temp2);

        LCDGoto(0, 1);
        LCDWriteStr("Pause     ");
        _delay(125000);
        
    }

    return (EXIT_SUCCESS);
}


