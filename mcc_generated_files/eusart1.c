/**
  EUSART1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    eusart1.c

  @Summary
    This is the generated driver implementation file for the EUSART1 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides APIs for EUSART1.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.77
        Device            :  PIC18F45K22
        Driver Version    :  2.1.0
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.05 and above
        MPLAB 	          :  MPLAB X 5.20
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

/**
  Section: Included Files
*/
#include <pic18.h>
#include <stdio.h>
#include "eusart1.h"
#include "pwm5.h"
#include "pin_manager.h"
#include "mcc.h"

/**
  Section: Macro Declarations
*/

#define EUSART1_TX_BUFFER_SIZE 8
#define EUSART1_RX_BUFFER_SIZE 8

/**
  Section: Global Variables
*/
volatile uint8_t eusart1TxHead = 0;
volatile uint8_t eusart1TxTail = 0;
volatile uint8_t eusart1TxBuffer[EUSART1_TX_BUFFER_SIZE];
volatile uint8_t eusart1TxBufferRemaining;

volatile uint8_t eusart1RxHead = 0;
volatile uint8_t eusart1RxTail = 0;
volatile uint8_t eusart1RxBuffer[EUSART1_RX_BUFFER_SIZE];
volatile eusart1_status_t eusart1RxStatusBuffer[EUSART1_RX_BUFFER_SIZE];
volatile uint8_t eusart1RxCount;
volatile eusart1_status_t eusart1RxLastError;

/**
  Section: EUSART1 APIs
*/
void (*EUSART1_TxDefaultInterruptHandler)(void);
void (*EUSART1_RxDefaultInterruptHandler)(void);

void (*EUSART1_FramingErrorHandler)(void);
void (*EUSART1_OverrunErrorHandler)(void);
void (*EUSART1_ErrorHandler)(void);

void EUSART1_DefaultFramingErrorHandler(void);
void EUSART1_DefaultOverrunErrorHandler(void);
void EUSART1_DefaultErrorHandler(void);

int manualSpeed = 1;
int unitTestCounter = 0;
bool atCheck = false;
bool muxSetup = false;
bool serverSetup = false;
int ipSetFlag = 0;
bool commandReceive = false;
bool commandReceive2 = false;
bool commandReceive3 = false;
bool commandReceive4 = false;
bool sensorsRead = false;
bool autoMode = false;

int step = 0;

void _step_(int stepDirection)
{
    if (step == 0){
        IO_RD4_SetLow();
        IO_RD5_SetHigh();
        IO_RD6_SetHigh();
        IO_RD7_SetHigh();
        
        step++;
        
    }
    
    else if (step == 1){
        IO_RD4_SetHigh();
        IO_RD7_SetHigh();
        IO_RD6_SetHigh();
        IO_RD5_SetLow();
        step++;
    }
    
    else if (step == 2){
        IO_RD4_SetHigh();
        IO_RD5_SetHigh();
        IO_RD6_SetLow();
        IO_RD7_SetHigh();
        step++;
    }
    
    else if (step == 3){
        IO_RD4_SetHigh();
        IO_RD7_SetLow();
        IO_RD6_SetHigh();
        IO_RD5_SetHigh();
        step=0;
    }
}

void _stepReverse_(int stepDirection)
{
    if (step == 0){
        IO_RD7_SetLow();
        IO_RD5_SetHigh();
        IO_RD6_SetHigh();
        IO_RD4_SetHigh();
        
        step++;
        
    }
    
    else if (step == 1){
        IO_RD4_SetHigh();
        IO_RD7_SetHigh();
        IO_RD5_SetHigh();
        IO_RD6_SetLow();
        step++;
    }
    
    else if (step == 2){
        IO_RD4_SetHigh();
        IO_RD6_SetHigh();
        IO_RD5_SetLow();
        IO_RD7_SetHigh();
        step++;
    }
    
    else if (step == 3){
        IO_RD7_SetHigh();
        IO_RD4_SetLow();
        IO_RD6_SetHigh();
        IO_RD5_SetHigh();
        step=0;
    }
}

void EUSART1_Initialize(void)
{
    // disable interrupts before changing states
    PIE1bits.RC1IE = 0;
    EUSART1_SetRxInterruptHandler(EUSART1_Receive_ISR);
    PIE1bits.TX1IE = 0;
    EUSART1_SetTxInterruptHandler(EUSART1_Transmit_ISR);
    // Set the EUSART1 module to the options selected in the user interface.

    // ABDOVF no_overflow; CKTXP async_noninverted_sync_fallingedge; BRG16 16bit_generator; WUE disabled; ABDEN disabled; DTRXP not_inverted; 
    BAUDCON1 = 0x08;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
    RCSTA1 = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave_mode; 
    TXSTA1 = 0x24;

    // 
    SPBRG1 = 0xCF;

    // 
    SPBRGH1 = 0x00;


    EUSART1_SetFramingErrorHandler(EUSART1_DefaultFramingErrorHandler);
    EUSART1_SetOverrunErrorHandler(EUSART1_DefaultOverrunErrorHandler);
    EUSART1_SetErrorHandler(EUSART1_DefaultErrorHandler);

    eusart1RxLastError.status = 0;

    // initializing the driver state
    eusart1TxHead = 0;
    eusart1TxTail = 0;
    eusart1TxBufferRemaining = sizeof(eusart1TxBuffer);

    eusart1RxHead = 0;
    eusart1RxTail = 0;
    eusart1RxCount = 0;

    // enable receive interrupt
    PIE1bits.RC1IE = 1;
}

bool EUSART1_is_tx_ready(void)
{
    return (eusart1TxBufferRemaining ? true : false);
}

bool EUSART1_is_rx_ready(void)
{
    return (eusart1RxCount ? true : false);
}

bool EUSART1_is_tx_done(void)
{
    return TXSTA1bits.TRMT;
}

eusart1_status_t EUSART1_get_last_status(void){
    return eusart1RxLastError;
}

uint8_t EUSART1_Read(void)
{
    uint8_t readValue  = 0;
    
    while(0 == eusart1RxCount)
    {
    }

    eusart1RxLastError = eusart1RxStatusBuffer[eusart1RxTail];

    readValue = eusart1RxBuffer[eusart1RxTail++];
    if(sizeof(eusart1RxBuffer) <= eusart1RxTail)
    {
        eusart1RxTail = 0;
    }
    PIE1bits.RC1IE = 0;
    eusart1RxCount--;
    PIE1bits.RC1IE = 1;

    return readValue;
}

void EUSART1_Write(uint8_t txData)
{
    while(0 == eusart1TxBufferRemaining)
    {
    }

    if(0 == PIE1bits.TX1IE)
    {
        TXREG1 = txData;
    }
    else
    {
        PIE1bits.TX1IE = 0;
        eusart1TxBuffer[eusart1TxHead++] = txData;
        if(sizeof(eusart1TxBuffer) <= eusart1TxHead)
        {
            eusart1TxHead = 0;
        }
        eusart1TxBufferRemaining--;
    }
    PIE1bits.TX1IE = 1;
}

char getch(void)
{
    return EUSART1_Read();
}

void putch(char txData)
{
    EUSART1_Write(txData);
}

void EUSART1_Transmit_ISR(void)
{

    // add your EUSART1 interrupt custom code
    if(sizeof(eusart1TxBuffer) > eusart1TxBufferRemaining)
    {
        TXREG1 = eusart1TxBuffer[eusart1TxTail++];
        if(sizeof(eusart1TxBuffer) <= eusart1TxTail)
        {
            eusart1TxTail = 0;
        }
        eusart1TxBufferRemaining++;
    }
    else
    {
        PIE1bits.TX1IE = 0;
    }
}

void EUSART1_Receive_ISR(void)
{
    
    eusart1RxStatusBuffer[eusart1RxHead].status = 0;

    if(RCSTA1bits.FERR){
        eusart1RxStatusBuffer[eusart1RxHead].ferr = 1;
        EUSART1_FramingErrorHandler();
    }

    if(RCSTA1bits.OERR){
        eusart1RxStatusBuffer[eusart1RxHead].oerr = 1;
        EUSART1_OverrunErrorHandler();
    }
    
    if(eusart1RxStatusBuffer[eusart1RxHead].status){
        EUSART1_ErrorHandler();
    } else {
        EUSART1_RxDataHandler();
    }
    
    // or set custom function using EUSART1_SetRxInterruptHandler()
}

void EUSART1_RxDataHandler(void){
    // use this default receive interrupt handler code
    eusart1RxBuffer[eusart1RxHead++] = RCREG1;
    
    if (unitTestCounter == 0 ) {
                
                if (atCheck ==false) {
                   if (eusart1RxBuffer[eusart1RxHead-1] == 'O') {
                        
                       atCheck = true;
                
                    } 
                } else if (eusart1RxBuffer[eusart1RxHead-1] == 'K') {
                    
                    atCheck = false;
                    unitTestCounter++;
                    
                } else {
                    atCheck = false;
                }
                
        } else if (unitTestCounter == 1) {
                if (muxSetup ==false) {
                   if (eusart1RxBuffer[eusart1RxHead-1] == 'O') {
                        
                       muxSetup = true;
                
                    } 
                } else if (eusart1RxBuffer[eusart1RxHead-1] == 'K') {
                    
                    muxSetup = false;
                    unitTestCounter++;
                    
                } else {
                    muxSetup = false;
                }
            } else if (unitTestCounter == 2) {
                if (serverSetup ==false) {
                   if (eusart1RxBuffer[eusart1RxHead-1] == 'O') {
                        
                       serverSetup = true;
                
                    } 
                } else if (eusart1RxBuffer[eusart1RxHead-1] == 'K') {
                    
                    serverSetup = false;
                    unitTestCounter++;
                    
                } else {
                    serverSetup = false;
                }
            } else if (unitTestCounter == 3) {
                if (ipSetFlag == 0) {
                   if (eusart1RxBuffer[eusart1RxHead-1] == '1') {
                        
                       ipSetFlag = 1;
                
                    } 
                } else if (eusart1RxBuffer[eusart1RxHead-1] == '9' && ipSetFlag == 1) {
                    
                    ipSetFlag = 2;
                    
                } else if (eusart1RxBuffer[eusart1RxHead-1] == '2' && ipSetFlag == 2) {
                    
                    ipSetFlag = 0;
                    unitTestCounter++;
                    
                } else {
                    ipSetFlag = 0;
                }
            } else if (unitTestCounter == 4) {
                if (commandReceive == false && commandReceive2 == false && commandReceive3 == false && commandReceive4 == false) {
                   if (eusart1RxBuffer[eusart1RxHead-1] == 'S') {
                        
                       commandReceive = true;
                       
                
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == 'D') {
                        commandReceive2 = true;
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == 'X') {
                        commandReceive3 = true;
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == 'Y') {
                        commandReceive4 = true;
                    }
                   
                   
                   
                } else if (commandReceive == true) {
                    
                    if (eusart1RxBuffer[eusart1RxHead-1] == '0') {
                    
                        manualSpeed = 1;
                        PWM5_LoadDutyValue(1023);
                    
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '1') {
                    
                        manualSpeed = 1;
                        PWM5_LoadDutyValue(500);
                    
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '2') {
                    
                        manualSpeed = 2;
                        PWM5_LoadDutyValue(330);
                    
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '3') {
                    
                        manualSpeed = 3;
                        PWM5_LoadDutyValue(170);
                    
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '4') {
                    
                        manualSpeed = 4;
                        PWM5_LoadDutyValue(0);
                    
                    }
                    commandReceive = false;
                    
                } else if (commandReceive2 == true){
                    if (eusart1RxBuffer[eusart1RxHead-1] == '1') {
                        
                        for (int i = 0; i < 12; i++) {
                            __delay_ms(50);
                          _step_(0);  
                        }
                        
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '2') {
                        
                        for (int i = 0; i < 24; i++) {
                            __delay_ms(50);
                          _step_(0);  
                        }
                        
                        
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '3') {
                        
                        for (int i = 0; i < 36; i++) {
                            __delay_ms(50);
                          _step_(0);  
                        }
                        
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '4') {
                        
                        for (int i = 0; i < 12; i++) {
                            __delay_ms(50);
                          _stepReverse_(0);  
                        }
                        
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '5') {
                        
                        for (int i = 0; i < 24; i++) {
                            __delay_ms(50);
                          _stepReverse_(0);  
                        }
                        
                        
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '6') {
                        
                        for (int i = 0; i < 36; i++) {
                            __delay_ms(50);
                          _stepReverse_(0);  
                        }
                        
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '7') {
                        
                        for (int i = 0; i < 6; i++) {
                            __delay_ms(50);
                          _step_(0);  
                        }
                        
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '8') {
                        
                        for (int i = 0; i < 6; i++) {
                            __delay_ms(50);
                          _stepReverse_(0);  
                        }
                        
                    }
                    
                    commandReceive2 = false;
                    
                } else if (commandReceive3 == true){
                    if (eusart1RxBuffer[eusart1RxHead-1] == '3') {                        
                        sensorsRead = true;
                        
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '4') {
                        
                        sensorsRead = true;
                        autoMode = true;
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '2') {
                        
                        sensorsRead = true;
                        autoMode = false;
                    } 
                    
                    commandReceive3 = false;
                    
                } else if (commandReceive4 == true){
                    if (eusart1RxBuffer[eusart1RxHead-1] == '0') {
                        
                        IO_RA0_SetLow();
                        
                    } else if (eusart1RxBuffer[eusart1RxHead-1] == '1') {
                        
                        IO_RA0_SetHigh();
                        
                    }  
                    
                    commandReceive4 = false;
                    
                } else {
                    commandReceive = false;
                    commandReceive2 = false;
                    commandReceive3 = false;
                    commandReceive4 = false;
                }
            }

    
    if(sizeof(eusart1RxBuffer) <= eusart1RxHead)
    {
        eusart1RxHead = 0;
    }
    eusart1RxCount++;
   
}

void EUSART1_DefaultFramingErrorHandler(void){}

void EUSART1_DefaultOverrunErrorHandler(void){
    // EUSART1 error - restart

    RCSTA1bits.CREN = 0;
    RCSTA1bits.CREN = 1;

}

void EUSART1_DefaultErrorHandler(void){
    EUSART1_RxDataHandler();
}

void EUSART1_SetFramingErrorHandler(void (* interruptHandler)(void)){
    EUSART1_FramingErrorHandler = interruptHandler;
}

void EUSART1_SetOverrunErrorHandler(void (* interruptHandler)(void)){
    EUSART1_OverrunErrorHandler = interruptHandler;
}

void EUSART1_SetErrorHandler(void (* interruptHandler)(void)){
    EUSART1_ErrorHandler = interruptHandler;
}

void EUSART1_SetTxInterruptHandler(void (* interruptHandler)(void)){
    EUSART1_TxDefaultInterruptHandler = interruptHandler;
}

void EUSART1_SetRxInterruptHandler(void (* interruptHandler)(void)){
    EUSART1_RxDefaultInterruptHandler = interruptHandler;
}

int EUSART1_GetUnitTestCounter(void) {
    
    return unitTestCounter;
}

bool EUSART1_GetSensorsBool(void) {
    
    return sensorsRead;
}

void EUSART1_ToggleSensorsBool(void) {
    
    sensorsRead = false;
}

bool EUSART1_GetAutoModeBool(void) {
    
    return autoMode;
}

/**
  End of File
*/
