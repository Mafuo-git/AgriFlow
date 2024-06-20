/*
 * File:   main.c
 * Author: Matheal & BRS & JMC
 *
 * Created on 19 May 2017
 */

#define USE_AND_MASKS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <xc.h>
#include "general.h"
#include "uart.h"
#include "i2c.h"


/*****/
// CONFIG1H
#pragma config FOSC = INTIO7    // Oscillator Selection bits (Internal oscillator block, CLKOUT function on OSC2)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (Power up timer enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = 3      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
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

    uint8_t reg_val, pout;                
    int16_t i, rssi;
    uint8_t txBuffer;
    uint8_t txBuffer1;
    char string[16];
    BOOL debut = 0;
    int myvar = 0;
    int myvar2 = 0;
    char buffer[50];

    
double t_fine;

typedef struct {
    unsigned short dig_T1;
    short dig_T2;
    short dig_T3;
    unsigned short dig_P1;
    short dig_P2;
    short dig_P3;
    short dig_P4;
    short dig_P5;
    short dig_P6;
    short dig_P7;
    short dig_P8;
    short dig_P9;
    unsigned char dig_H1;
    short dig_H2;
    unsigned char dig_H3;
    short dig_H4;
    short dig_H5;
    char dig_H6;
} BME280_CompParams;

BME280_CompParams compParams;

void I2C_Write_Register(unsigned char reg, unsigned int value, unsigned int address) {
    i2c_start();
    i2c_write((address << 1) | 0); // Send address with write condition
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

void BME280_Init(void) {
    i2c_start();
    i2c_write(0xEC); // I2C address of BME280 in write mode
    i2c_write(0xF2); // Humidity control register
    i2c_write(0x01); // Humidity oversampling x1
    i2c_stop();

    i2c_start();
    i2c_write(0xEC);
    i2c_write(0xF4); // Control measurement register
    i2c_write(0x27); // Temp and pressure oversampling x1, mode normal
    i2c_stop();

    i2c_start();
    i2c_write(0xEC);
    i2c_write(0xF5); // Config register
    i2c_write(0xA0); // Standby time 1000ms, filter off
    i2c_stop();
}

void BME280_Read_Compensation_Params(void) {
    unsigned int data[6];

    i2c_start();
    i2c_write(0xEC); // Adresse I2C du BME280 en écriture
    i2c_write(0x88); // Adresse du registre de compensation
    i2c_stop();

    i2c_start();
    i2c_write(0xED); // Adresse I2C du BME280 en lecture
    for (int i = 0; i < 5; i++) {
        data[i] = i2c_read();
        i2c_ACK();
    }
    data[5] = i2c_read();
    i2c_NAK();
    i2c_stop();

    // Convertir les données lues en valeurs de compensation
    compParams.dig_T1 = (unsigned short)((data[1] << 8) | data[0]);
    compParams.dig_T2 = (short)((data[3] << 8) | data[2]);
    compParams.dig_T3 = (short)((data[5] << 8) | data[4]);
}

long BME280_Read_Temperature(void) {
    unsigned char msb, lsb, xlsb;
    long adc_T;

    i2c_start();
    i2c_write(0xEC); // Adresse I2C du BME280 en écriture
    i2c_write(0xFA); // Adresse du registre de température MSB
    i2c_stop();

    i2c_start();
    i2c_write(0xED); // Adresse I2C du BME280 en lecture
    msb = i2c_read();
    i2c_ACK();
    lsb = i2c_read();
    i2c_ACK();
    xlsb = i2c_read();
    i2c_NAK();
    i2c_stop();

    //LCDWriteHexa(10,1,msb);
    //LCDWriteHexa(12,1,lsb);
    //LCDWriteHexa(14,1,xlsb);
    
    //printf("msb: %02X\n", msb);
    //printf("lsb: %02X\n", lsb);
    //printf("xlsb: %02X\n", xlsb);

    adc_T = (long)((msb << 12) | (lsb << 4) | (xlsb >> 4));
    //printf("adc_T: %ld\n", adc_T);

    return adc_T;
}

double BME280_Compensate_Temperature(long adc_T) {
    /*long var1, var2, T;
    var1 = (((double)adc_T) / 16384.0 - ((double)compParams.dig_T1) / 1024.0) * ((double)compParams.dig_T2);
    var2 = ((((double)adc_T) / 131072.0 - ((double)compParams.dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)compParams.dig_T1) / 8192.0)) * ((double)compParams.dig_T3);
    
    unsigned char var1_[16];
        
    sprintf(var1_, "%f", var1);
    LCDGoto(0, 0);
    LCDWriteStr(var1_);
    
    unsigned char var2_[16];
        
    sprintf(var2_, "%f", var2);
    LCDGoto(8, 0);
    LCDWriteStr(var2_);
    
    T = (var1 + var2) / 5120.0;
    return T;
    */
   /* long var1, var2, t_fine, T;
    var1 = (((adc_T >> 3) - ((long)compParams.dig_T1 << 1)) * ((long)compParams.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - (long)compParams.dig_T1) * ((adc_T >> 4) - (long)compParams.dig_T1)) >> 12) * (long)compParams.dig_T3) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return (double)T / 100.0;*/
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;double t_fine;

    var1 = (((double)adc_T) / 16384.0 - ((double)compParams.dig_T1) / 1024.0);
    var1 = var1 * ((double)compParams.dig_T2);
    var2 = (((double)adc_T) / 131072.0 - ((double)compParams.dig_T1) / 8192.0);
    var2 = (var2 * var2) * ((double)compParams.dig_T3);
    t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    /*if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }*/

    return temperature;
    
    
}
    
void __interrupt() SerialRxPinInterrupt() {                         // interrupt has high priority
//    void interrupt low_priority SerialRxPinInterrupt() {         // interrupt has low priority
    if (PIR1bits.RCIF == 1) {               //check if the interrupt is caused by RX pin
        if (RCSTA1bits.FERR) {
            UARTWriteStrLn("Framing Error");
            UARTReadByte();
        }
        else if (RCSTA1bits.OERR) {
            UARTWriteStrLn("Overrun Error");
            RCSTA1bits.CREN = CLEAR;
            asm("NOP");
            RCSTA1bits.CREN = SET;
        }
        else {
            if(debut == 0){
                txBuffer = UARTReadByte();
                //PIR1bits.RCIF = 0;           // useless: the flag is automatically cleared when RCREG is read
                }
            else{
                txBuffer = UARTReadByte();
                //PIR1bits.RCIF = 0;            // useless: the flag is automatically cleared when RCREG is read
                }
        }
    }
}
    
    
int main(int argc, char** argv) {
    
    
    INT8_T temp, temp2, hygro1, hygro2, watt1, watt2, temp_soil1, temp_soil2;                             // temperature (1 byte)
    char string[16];
    char degre[2] = {0xDF, '\0'};            // ASCII code for ° symbol, not known by XC8 compiler, see HD44780 datasheet
    

    i2c_init();                             // configuration de l'interface I2C
    BME280_Init();
    I2C_Write_Register(0x05, 0x0697, INA230_ADDRESS); // calibration du current
    _delay(12500);
     
     BME280_Read_Compensation_Params();
    
    forever{
        CLRWDT();
    OSCCONbits.IRCF = 5;
    UARTInit();            // init UART @ 57600 bps
    //i2c_init();
    
    ei();
    _delay(300);

    debut = 1;
    
    UARTWriteStrLn("mac reset 868");
    while (txBuffer != (UINT8_T)'o'){
       }
   
    txBuffer = CLEAR;
        
    UARTWriteStrLn("mac set appkey 9BAA4A6C8E16BB432D4966E34C0F8573");
    while (txBuffer != (UINT8_T)'o'){
        }
      
    txBuffer = CLEAR;
        
    UARTWriteStrLn("mac set appeui 23DF314F12FDFE14");
    while (txBuffer != (UINT8_T)'o'){
        }
    txBuffer = CLEAR;
        
    UARTWriteStrLn("mac set deveui 70B3D57ED0067ED6");
    while (txBuffer != (UINT8_T)'o'){
        }
      
    txBuffer = CLEAR;
        
    UARTWriteStrLn("mac save");
    while (txBuffer != (UINT8_T)'o'){
        }
        
    txBuffer = CLEAR;
        
    UARTWriteStrLn("mac join otaa");
    while (txBuffer != (UINT8_T)'a'){
        if (txBuffer == (UINT8_T)'d') {
            __asm("RESET");
            }
        }
        
    txBuffer = CLEAR;

    myvar = 0;

        temp = 0;
        temp2 = 0;

        _delay(125000);                         // wait for 125000 Tcy = 125000 * 4us = 0.5 s
        
        /*-------------------------------------*/
        /* Debut données hygrométrique (sonde) */
        
        i2c_start();                                    // send start condition
        i2c_write((HYGRO_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x00) ;                               // select current register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((HYGRO_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
        hygro1 = i2c_read();
        i2c_ACK();
        hygro2 = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();                                     // send stop condition
        
        i2c_start();
//        temp = abs(i2c_read());                         // read temperature (only 0 to 99 degrés Celsius)
        i2c_write((HYGRO_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x09) ;                               // select current register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((HYGRO_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
        
        INT8_T check_mesure = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();
        
        while (check_mesure != 0){
            _delay(5);
        }
        
        i2c_start();
        i2c_write((HYGRO_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x00) ;                               // select current register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((HYGRO_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
        
        hygro1 = i2c_read();
        i2c_ACK();
        hygro2 = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();                                     // send stop condition
        
        /* Fin données hygrométrie (sonde) */
        
        /* Debut données temperature sonde */
        
        i2c_start();                                    // send start condition
        i2c_write((HYGRO_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x05) ;                               // select temperature register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((HYGRO_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
        temp_soil1 = i2c_read();
        i2c_ACK();
        temp_soil2 = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();                                     // send stop condition
        
        i2c_start();
//        temp = abs(i2c_read());                         // read temperature (only 0 to 99 degrés Celsius)
        i2c_write((HYGRO_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x09) ;                               // select get_busy register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((HYGRO_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
        
        check_mesure = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();
        
        while (check_mesure != 0){
            _delay(5);
        }
        
        i2c_start();
        i2c_write((HYGRO_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x05) ;                               // select temperature register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((HYGRO_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
        
        temp_soil1 = i2c_read();
        i2c_ACK();
        temp_soil2 = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();                                     // send stop condition
        
        /* Fin données température sonde */
        
        /* Fin données sonde */
        /*-------------------------------------*/
        /* Debut données wattmètre (INA230) */
        
        i2c_start();                                    // send start condition
        i2c_write((INA230_ADDRESS << 1) | I2C_WRITE);     // send to slave 7-bit address (1001 101) + WR (0)
        i2c_write(0x04) ;                               // select current register
        i2c_repStart();                                 // send repeated start condition
        i2c_write((INA230_ADDRESS << 1) | I2C_READ);      // send to slave 7-bit address (1001 101) + RD (1)
//        temp = abs(i2c_read());                         // read temperature (only 0 to 99 degrés Celsius)
        watt1 = i2c_read();
        i2c_ACK();
        watt2 = i2c_read();
        i2c_NAK(); // send a NAK (last read)
        i2c_stop();                                     // send stop condition

        //LCDWriteHexa(7, 0, watt1);
        //LCDWriteHexa(9, 0, watt2);
        
        /* Fin données wattmètre (INA230) */
        /*------------------------------------*/
        /* Debut données température (BME280) */
        
        long adc_T = BME280_Read_Temperature(); // Lire la température brute

        double temp_air = BME280_Compensate_Temperature(adc_T); // Compenser la température
        
        unsigned long temp_32bit = (unsigned long)(temp_air * 1024 * 1024); // Convertir la valeur double en une valeur entière sur 24 bits
        unsigned char temp_MSB = (temp_32bit >> 16) & 0xFF; // Extraire le MSB (8 bits de poids fort)
        unsigned char temp_MidSB = (temp_32bit >> 8) & 0xFF; // Extraire les bits du milieu (8 bits)
        unsigned char temp_LSB = temp_32bit & 0xFF; // Extraire le LSB (8 bits de poids faible)


        
        //LCDWriteHexa(0, 0, temperature);
        
        float temp_celsius = (float)temp_air;
        
        /* Fin données température (BME280) */
        
        /* ----------------------------------- */
        
        /* 
         * Debut envoi données suivant la trame
         */
        
        // Send : hygro1, hygro2, 00, 00, 00, 00, 00, 00, temp_soil1, temp_soil2, temp_MSB, temp_MidSB
        // Si temp_air sur 8 bit, utiliser temp_MSB, temp_MidSB, temp_LSB
        
        
        /*
         * Fin envoi données suivant la trame
         */
        
        
        
        _delay(125000);
        
        
        myvar = 0x00;
        myvar2 = 0x00;
        
        int myvar3 = 0x00;
        int myvar4 = 0x00;
        
        int myvar5 = 0x00;
        int myvar6 = 0x00;

        sprintf(buffer, "mac tx uncnf 1 %x%x", watt1,watt2);
        UARTWriteStrLn(buffer);
        while (UARTReadByte() != (UINT8_T)'m'){
        }
        while (txBuffer != 0){
            txBuffer = 0;
        } 

        
        int compteur = 0;
        int bit_vanne = 0;
       // while (compteur < 7 && bit_vanne==0){
            SLEEP();
            CLRWDT();
            ++compteur;
            bit_vanne = 0;
        //}
    }
}