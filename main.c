/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.77
        Device            :  PIC18F45K22
        Driver Version    :  2.00
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
#define LCD_ADDRESS 0x27

#define LCD_COLS 16
#define LCD_ROWS 2

#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <math.h>
#include "i2c.h"
#include "lcd.h"
#include<string.h>



/*
                         Main application
 */
void sendData(float temp, int lux, float distanceC, float distanceL, float distanceR);
void Azo_Interrupt();
void i2cStart();
void i2cRepeatedStart();
void i2cStop();
unsigned i2cRead(unsigned i2cAddress, unsigned registerAdress);
int timeout = 1000;
void sendPulse1();
void sendPulse2();
void sendPulse3();
void dcMotor(int speeed);
void _stepMotor_(int stepDirection);
void _stepMotorReverse_(int stepDirection);
void initialiseTCP();

bool isAuto = false;
bool isOn = false;
unsigned int touchBytesAzo;
bool azoTouchPollManual();
bool azoTouchPollAuto();
void printOn();
void printAuto();
manualSpeed = 1;
int stepperPosition = 0;
bool stepperCW = true;

//flag variables:
bool autoFirst = true;
bool manualFirst = true;

//sensor variables:
//Light Sensors:
float adcValLight = 0;
float luxVal[2];
float voltageVal = 0.0;
float resistorVal = 0.0;
float finalLux = 0.0;

//Temperature Sensors:
float temperatureVal[2];
long double A = 0.00112530885212;
long double B = 0.000234711863267;
long double C = 0.000000085663516;
    
    
    //US Sensor:
float distance = 0.0;
int time = 0;
float val[10];
char valTemp[10];
char valDistance[10];

int step = 0;

//variables for wifi communication
float guiTemp, guiLux, guiDistanceL, guiDistanceC, guiDistanceR;


void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
//    INTCON_SetInterruptHandler (Azo_Interrupt());
    ADC_Initialize();
    I2C2_Initialize();
    
    PWM5_LoadDutyValue(1023);
    TMR3_StopTimer();
    
    __delay_ms(1000);
    LCD_init(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
    LCD_backlight(true);
    LCD_clear();
    LCD_send_string("System Start", 1);
    __delay_ms(5000);

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
    
    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
//    MSSP2_SetInterruptHandler (Azo_Interrupt());
//    SSP_SetInterruptHandler (Azo_Interrupt());
    
    initialiseTCP();
    checkTCP = false;
    
    LCD_clear();
    LCD_send_string("TCP Server \n  192.168.4.1", 1);
    __delay_ms(2500);
    
    while (1)
    {
        if (!isOn) {
            LCD_clear();
            LCD_send_string("Fan on Standby", 1);
            __delay_ms(1000);
            touchBytesAzo = i2cRead(0xC8, 0x03);
            if (touchBytesAzo == 7)
            {
                isOn = true;
                LCD_clear();
                manualSpeed = 1;
                dcMotor(manualSpeed);
                LCD_send_string("Fan On", 1);
                __delay_ms(1000);
            } 
            
        }
        
        if (!isAuto && isOn) {
            
            //dcMotor(manualSpeed);
            //manualSpeed = 1;
            LCD_clear();
            LCD_send_string("Manual mode", 1);
            __delay_ms(1000);
            
            printOn();
            
            
        } else if (isAuto && isOn) {

            //printf("Here4");
            LCD_clear();
            LCD_send_string("Auto mode", 1);
            __delay_ms(1000);
        
            printAuto(); 
        } // end if isAuto 
        
        //reestablish TCP to check
        if (checkTCP) {
            printf("AT+CIPSERVER=1,222\r\n");
            checkTCP = false;
        }
        
        if (EUSART1_GetSensorsBool()) 
        {
            sendData(guiTemp, guiLux, guiDistanceC, guiDistanceL, guiDistanceR);
        }
        
    }
}

void dcMotor(int speeed)
{
    if (speeed == 1)
    {
        PWM5_LoadDutyValue(500);
    }
    if (speeed == 2)
    {
        PWM5_LoadDutyValue(330);
    }
    if (speeed == 3)
    {
        PWM5_LoadDutyValue(170);
    }
    if (speeed == 4)
    {
        PWM5_LoadDutyValue(0);
    }
}

void sendData(float temp, int lux, float distanceC, float distanceL, float distanceR)
{
    printf("AT+CIPSEND=0,26\r\n");
    __delay_ms(500);

    //float (3 digits), four decimal, float (4 digits))
    printf ("S%04.1f%06.1f%05.3f%05.3f%05.3f\r\n", temp, lux, distanceC, distanceL, distanceR);

    EUSART1_ToggleSensorsBool();
}

bool azoTouchPollManual()
{
    touchBytesAzo = i2cRead(0xC8, 0x03);
            
    if (touchBytesAzo == 7)
    {
        isOn = false;
        return false;
    } else if (touchBytesAzo == 3) {
        isAuto = true;
        LCD_clear();
        LCD_send_string("Auto mode", 1);
        __delay_ms(1000);
        return false;
    } else if (touchBytesAzo == 5) {
        manualSpeed += 1;
        if (manualSpeed>4)
            manualSpeed = 1;
        dcMotor(manualSpeed);
    } else if ( touchBytesAzo == 9 ) {
        
        if (stepperCW) {
            for(int i =0;i<6;i++)
            {
                __delay_ms(50);
                _stepMotor_(0);
            }
            
            stepperPosition++;
            if (stepperPosition == 2) {
                stepperCW = false;
            }
        } else {
            for(int i =0;i<6;i++)
            {
                __delay_ms(50);
                _stepMotorReverse_(0);
            }
            
            stepperPosition--;
            if (stepperPosition == -2) {
                stepperCW = true;
            }
        }
    }
    return true;
}

bool azoTouchPollAuto()
{
    touchBytesAzo = i2cRead(0xC8, 0x03);
        
    if (touchBytesAzo == 7)
    {
        isOn = false;
        return false;
    } else if (touchBytesAzo == 3) {
        isAuto = false;
        LCD_clear();
        LCD_send_string("Manual mode", 1);
        __delay_ms(1000);
        return false;
    }
    return true;
}

void printOn()
{
    if (!azoTouchPollManual())
        return;
    
    //displaying of motor speed
    if (manualSpeed == 1) {
        LCD_clear();
        LCD_send_string("Speed 1", 1);
        __delay_ms(1000);
    } else if (manualSpeed == 2) {
        LCD_clear();
        LCD_send_string("Speed 2", 1);
        __delay_ms(1000);
    } else if (manualSpeed == 3) {
        LCD_clear();
        LCD_send_string("Speed 3", 1);
        __delay_ms(1000);
    } else if (manualSpeed == 4) {
        LCD_clear();
        LCD_send_string("Speed 4", 1);
        __delay_ms(1000);
    }
            
    if (!azoTouchPollManual())
        return;
}

void printAuto()
{
    if (EUSART1_GetSensorsBool()) 
    {
        sendData(guiTemp, guiLux, guiDistanceC, guiDistanceL, guiDistanceR);
    }
    
    if (!azoTouchPollAuto())
        return;
        
    
    adcValLight = ADC_GetConversion(channel_AN1);
    voltageVal = adcValLight/1023 * 3.3;
    resistorVal = voltageVal*10000/(3.3 - voltageVal);
    double tempLux = pow((double)resistorVal,4/3)*10;
    luxVal[0] = 10000/tempLux*100*5;
    
    adcValLight = ADC_GetConversion(channel_AN2);
    voltageVal = adcValLight/1023 * 3.3;
    resistorVal = voltageVal*10000/(3.3 - voltageVal);
    double tempLux = pow((double)resistorVal,4/3)*10;
    luxVal[1] = 10000/tempLux*100*5;
    finalLux = (luxVal[0] + luxVal[1]) / 2;
    guiLux = finalLux;
    
    float val[10];
    sprintf(val,"%.1f",finalLux);
    char strLux[]="Light (Lux)\n";
    strcat(strLux,val);
    LCD_clear();
    LCD_send_string(strLux, 1);
    __delay_ms(1000);
    
    if (!EUSART1_GetAutoModeBool()) {
        if (finalLux < 30.0) {
            IO_RA0_SetHigh();
        } else {
            IO_RA0_SetLow();
        }
    }
    
    if (!azoTouchPollAuto())
        return;
    
    if (EUSART1_GetSensorsBool()) 
    {
        sendData(guiTemp, guiLux, guiDistanceC, guiDistanceL, guiDistanceR);
    }
    
    //Temperature Sensor:
    float adcValTemp = ADC_GetConversion(channel_AN3);
    double v_o1 = adcValTemp /1023 *3.3;
    float R1 = v_o1*10000/(3.3-v_o1);  
    temperatureVal[0] = 1 / (A + B * log(R1) + C * pow(log(R1), 3)) - 273;
        
    float adcValTemp = ADC_GetConversion(channel_AN4);
    double v_o1 = adcValTemp /1023 *3.3;
    float R1 = v_o1*10000/(3.3-v_o1);
    temperatureVal[1] = 1 / (A + B * log(R1) + C * pow(log(R1), 3)) - 273;
    
    float finalTemperature = (temperatureVal[0]+temperatureVal[1])/2;
    guiTemp = finalTemperature;
    
    char valTemp[3];
    sprintf(valTemp,"%.1f",finalTemperature);
    char strTemp[]="Temperature \337C\n";
//      LCD_clear();
//      LCD_send_string("Temperature (\337C)",1);
    __delay_ms(1000);
    strcat(strTemp,valTemp);
    LCD_clear();
    LCD_send_string(strTemp, 1);
    __delay_ms(1000);
        
    if (!EUSART1_GetAutoModeBool()) {
        if (finalTemperature>=20 && finalTemperature<25)
        {
            manualSpeed = 1;
            dcMotor(1);
            LCD_clear();
            LCD_send_string("Speed 1", 1);
            __delay_ms(1000);
        }
        else if(finalTemperature>=25 && finalTemperature<28)
        {
            manualSpeed = 2;
            dcMotor(2);
            LCD_clear();
            LCD_send_string("Speed 2", 1);
            __delay_ms(1000);
        }
        else if(finalTemperature>=28 && finalTemperature<30)
        {
            manualSpeed = 3;
            dcMotor(3);
            LCD_clear();
            LCD_send_string("Speed 3", 1);
            __delay_ms(1000);
        }
        else if (finalTemperature>=30)
        {
            manualSpeed = 4;
            dcMotor(4);
            LCD_send_string("Speed 4", 1);
            __delay_ms(1000);
        }
    }
    
    if (!azoTouchPollAuto())
        return;

    if (EUSART1_GetSensorsBool()) 
    {
        sendData(guiTemp, guiLux, guiDistanceC, guiDistanceL, guiDistanceR);
    }
    
    bool nearFan = true;
    //Ultrasonic Sensor1:
    while (nearFan) {
        sendPulse1();
        while (PORTBbits.RB2==0);
        TMR1 = 0;
        TMR1ON = 1;
        while (PORTBbits.RB2 == 1 && !TMR1IF);
        time = TMR1;
    //        TMR1ON = 1;
        TMR1ON = 0;

        distance  = time/117.0/100.0;

        guiDistanceC = distance;

        sprintf(valDistance,"%.3f",distance);
        char str[]="Distance 1(m)\n";
        strcat(str,valDistance);
        LCD_clear();
        LCD_send_string(str, 1);
        __delay_ms(1000);
        if (distance<=.15)
        {
            PWM5_LoadDutyValue(1023);
            LCD_clear();
            LCD_send_string("Detected \n Front", 1);
            __delay_ms(1000);

        } else {
           
            nearFan = false;
            dcMotor(manualSpeed);
            break;
        }

        if (!azoTouchPollAuto())
            return;
    
    }
    
    if (EUSART1_GetSensorsBool()) 
    {
        sendData(guiTemp, guiLux, guiDistanceC, guiDistanceL, guiDistanceR);
    }
    
     //Ultrasonic Sensor 2:
    nearFan = true;
    
    
    while (nearFan) {
    
        sendPulse2();
        while (PORTBbits.RB5==0);
        TMR1 = 0;
        TMR1ON = 1;
        while (PORTBbits.RB5 == 1 && !TMR1IF);
        time = TMR1;
    //        TMR1ON = 1;
        TMR1ON = 0;

        distance  = time/117.0/100.0;

        guiDistanceL = distance;

        sprintf(valDistance,"%.3f",distance);
        char str[]="Distance 2(m)\n";
        strcat(str,valDistance);
        LCD_clear();
        LCD_send_string(str, 1);
        __delay_ms(1000);
        if (distance<=.15)
        {
            PWM5_LoadDutyValue(1023);
            LCD_clear();
            LCD_send_string("Detected \n Left", 1);
            __delay_ms(1000);

        } else {
            dcMotor(manualSpeed);
            nearFan = false;
            break;
        }

        if (!azoTouchPollAuto())
            return;
    
    }    
    
    if (EUSART1_GetSensorsBool()) 
    {
        sendData(guiTemp, guiLux, guiDistanceC, guiDistanceL, guiDistanceR);
    }
    
     //Ultrasonic Sensor 3:
    nearFan = true;
    
    LCD_clear();
    LCD_send_string("Distance Three", 1);
    __delay_ms(1000);
    
    while (nearFan) {
    
        sendPulse3();
        while (PORTBbits.RB7==0);
        TMR1 = 0;
        TMR1ON = 1;
        while (PORTBbits.RB7 == 1 && !TMR1IF);
        time = TMR1;
    //        TMR1ON = 1;
        TMR1ON = 0;

        distance  = time/117.0/100.0;

        guiDistanceR = distance;

        sprintf(valDistance,"%.3f",distance);
        char str[]="Distance 3(m)\n";
        strcat(str,valDistance);
        LCD_clear();
        LCD_send_string(str, 1);
        __delay_ms(1000);
        if (distance<=.15)
        {
            PWM5_LoadDutyValue(1023);
            LCD_clear();
            LCD_send_string("Detected \n Right", 1);
            __delay_ms(1000);

        } else {
                dcMotor(manualSpeed);
                nearFan = false;
                break;
        }
        
        
    if (!azoTouchPollAuto())
        return;
        
    }
    
    if (EUSART1_GetSensorsBool()) 
    {
        sendData(guiTemp, guiLux, guiDistanceC, guiDistanceL, guiDistanceR);
    }
}


void sendPulse1()
{
    LATBbits.LATB1 = 1;
    __delay_us(10);
    LATBbits.LATB1 = 0;
}

void sendPulse2()
{
    LATBbits.LATB4 = 1;
    __delay_us(10);
    LATBbits.LATB4 = 0;
}

void sendPulse3()
{
    LATBbits.LATB6 = 1;
    __delay_us(10);
    LATBbits.LATB6 = 0;
}

void i2cStart()
{
    SSP2CON2bits.SEN = 1;
    timeout = 1000;
    while (PIR3bits.SSP2IF == 0 && timeout>0)
        timeout--;
    PIR3bits.SSP2IF = 0;
    return ;
}
void i2cRepeatedStart()
{
    SSP2CON2bits.RSEN = 1;

    timeout = 1000;    
    while (PIR3bits.SSP2IF == 0 && timeout>0)        
        timeout--;
    PIR3bits.SSP2IF = 0;
    return ;
    
}
void i2cStop()
{
    SSP2CON2bits.PEN = 1;
    
    timeout = 1000;    
    while (PIR3bits.SSP2IF == 0 && timeout>0)        
        timeout--;
    PIR3bits.SSP2IF = 0;
    
    timeout = 1000;
	while (SSP2CON2bits.PEN == 1 && timeout>0)
        timeout--;

//    __delay_ms(100);
    
    return ;
}
unsigned i2cRead(unsigned i2cAddress, unsigned registerAdress)
{
    i2cStart();
    
    SSP2BUF = i2cAddress;
    while (SSP2STATbits.BF2 == 1);
    timeout = 1000;    
    while (PIR3bits.SSP2IF == 0 && timeout>0)        
        timeout--;
    PIR3bits.SSP2IF = 0;
    
    SSP2BUF = registerAdress;
    while (SSP2STATbits.BF2 == 1);
    timeout = 1000;    
    while (PIR3bits.SSP2IF == 0 && timeout>0)        
        timeout--;
    PIR3bits.SSP2IF = 0;
    
    i2cRepeatedStart();
    
    SSP2BUF = i2cAddress+1;
    timeout = 1000;    
    while (PIR3bits.SSP2IF == 0 && timeout>0)        
        timeout--;
    PIR3bits.SSP2IF = 0;
    
    SSP2CON2bits.RCEN = 1;
    timeout = 1000;    
    while (PIR3bits.SSP2IF == 0 && timeout>0)        
        timeout--;
    PIR3bits.SSP2IF = 0;
    
    SSP2CON2bits.ACKDT = 1;
    SSP2CON2bits.ACKEN = 1;
    timeout = 1000;    
    while (SSP2CON2bits.ACKEN == 1 && timeout>0)
        timeout--;
//    while (PIR1bits.SSP1IF == 0 && timeout>0)        
//        timeout--;
//    PIR1bits.SSP1IF = 0;
    
    i2cStop();
    
    return SSP2BUF;
}

void _stepMotor_(int stepDirection)
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

void _stepMotorReverse_(int stepDirection)
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

void initialiseTCP() {
    
    printf("AT\r\n");        
    while (EUSART1_GetUnitTestCounter() != 1) {   
    }
    printf("AT+CIPMUX=1\r\n");
    while (EUSART1_GetUnitTestCounter() != 2) {    
    }
    printf("AT+CIPSERVER=1,222\r\n");
    __delay_ms(500);
    while (EUSART1_GetUnitTestCounter() != 3) { 
        __delay_ms(1000);
        if (EUSART1_GetUnitTestCounter() != 3) {
            //printf("AT+RST\r\n");
            __delay_ms(100);
            printf("AT+CIPMUX=1\r\n");
            printf("AT+CIPSERVER=1,222\r\n");      
        }
    }
    __delay_ms(200);
    printf("AT+CIFSR\r\n"); 
    
    __delay_ms(500);
    printf("AT+CIPMUX=1\r\n");
    printf("AT+CIPSERVER=1,222\r\n");
    
    TMR3_StartTimer();
    
}


/**
 End of File
*/