/*
 * File:   RXsingle_LoRa.c
 * Authors: BRS & JMC
 *
 * Created on 31 May 2017
 */

#define USE_AND_MASKS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "general.h"
#include "uart.h"
#include "i2c.h"
#include "lcd.h"
#include <xc.h>



/*****/
// CONFIG1H
#pragma config FOSC = INTIO7    // Oscillator Selection bits (Internal oscillator block, CLKOUT function on OSC2)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (Power up timer enaabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
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
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
//#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
//#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
//#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
//#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
//#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
//#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

void __interrupt() SerialRxPinInterrupt() {
    if (PIR1bits.RCIF == 1) {  // Vérifier si l'interruption est causée par la broche RX
        if (RCSTA1bits.FERR) {
            UARTWriteStr("Framing Error ");
            RCSTA1bits.SPEN = 0;  // Clear the error
            RCSTA1bits.SPEN = 1;
        } else if (RCSTA1bits.OERR) {
            UARTWriteStr("Overrun Error ");
            RCSTA1bits.CREN = 0;  // Clear the error
            RCSTA1bits.CREN = 1;
        } else {
            uint8_t receivedChar = RCREG1; // Read the character immediately to avoid overrun
            UARTWriteByteDec(receivedChar);  // Display the received character as ASCII
        }
        PIR1bits.RCIF = 0; // Clear the RX interrupt flag
    }
}

void ouvrir_vanne(){
    TRISCbits.TRISC1 = 1;      // vers IN1 du pont en H 
    TRISCbits.TRISC2 = 0;      // vers IN2 du pont en H
    __delay_ms(250);
    TRISCbits.TRISC1 = 0;
}
    
void fermer_vanne(){
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 1;
    __delay_ms(250);
    TRISCbits.TRISC2 = 0;
}

int main(int argc, char** argv) {
    
         // to store the number of bytes received
    uint8_t i;
    
    UARTInit(9600);            // init UART @ 9600 bps
    
   OSCCON = 0x72; // Configurer l'oscillateur à 16 MHz
    i2c_init(); // Initialiser l'I2C

    uint8_t humidity;
    forever { 
        SONDE_ReadHumidity(&humidity); // Lire les données de la sonde

        // Afficher l'humidité lue (par exemple, via LCD ou UART)
        LCDClear();
        LCDGoto(0, 0);
        LCDWriteStr("Humidite:");
        LCDGoto(0, 1);
        LCDWriteInt(humidity);
        LCDPutChar('%');

        __delay_ms(1000); // Attendre 1 seconde avant la prochaine lecture
    }       // end of loop forever
}
