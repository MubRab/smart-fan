/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef LCD_H
#define	LCD_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>

// Commands
#define LCD_CLEARDISPLAY 0x01        // 0b00000001
#define LCD_RETURNHOME 0x02          // 0b00000010
#define LCD_ENTRYMODESET 0x04        // 0b00000100
#define LCD_DISPLAYCONTROL 0x08      // 0b00001000
#define LCD_CURSORSHIFT 0x10         // 0b00010000
#define LCD_FUNCTIONSET 0x20         // 0b00100000
#define LCD_SETCGRAMADDR 0x40        // 0b01000000
#define LCD_SETDDRAMADDR 0x80        // 0b10000000

// Flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02           // 0b00000010
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAYON 0x04           // 0b00000100
#define LCD_DISPLAYOFF 0x00          // 0b00000000
#define LCD_CURSORON 0x02            // 0b00000010
#define LCD_CURSOROFF 0x00           // 0b00000000
#define LCD_BLINKON 0x01             // 0b00000001
#define LCD_BLINKOFF 0x00            // 0b00000000

// Flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08         // 0b00001000
#define LCD_CURSORMOVE 0x00          // 0b00000000
#define LCD_MOVERIGHT 0x04           // 0b00000100
#define LCD_MOVELEFT 0x00            // 0b00000000

// Flags for function set
#define LCD_8BITMODE 0x10            // 0b00010000
#define LCD_4BITMODE 0x00            // 0b00000000
#define LCD_2LINE 0x08               // 0b00001000
#define LCD_1LINE 0x00               // 0b00000000
#define LCD_5x10DOTS 0x04            // 0b00000100
#define LCD_5x8DOTS 0x00             // 0b00000000

// Flags for backlight
#define LCD_BACKLIGHT 0x08           // 0b00001000
#define LCD_NOBACKLIGHT 0x00         // 0b00000000

// Lines
#define LCD_LINE1 0x80               // 0b10000000
#define LCD_LINE2 0xC0               // 0b11000000
//#define LCD_LINE1 0x00               // 0b10000000
//#define LCD_LINE2 0x40               // 0b11000000     
#define LCD_LINE3 0x94               // 0b10010100
#define LCD_LINE4 0xD4               // 0b11010100
    
#define En 0b00000100                // Enable bit
#define Rw 0b00000010                // Read/Write bit
#define Rs 0b00000001                // Register select bit

/**
 * LCD module intialization.
 * 
 * @param address I2C 7bit address of the LCD.
 * @param cols Number of columns.
 * @param rows Number of rows.
 */
void LCD_init(uint8_t address, uint8_t cols, uint8_t rows);

/**
 * Clear the content of the LCD.
 */
void LCD_clear(void);

/**
 * Turn LCD's backlight on or off.
 * 
 * @param on Whether to turn the backlight on or off.
 */
void LCD_backlight(bool on);

/**
 * Send a command to the LCD.
 * 
 * @param command The command.
 */
void LCD_send_cmd(uint8_t command);

/**
 * Send data to the LCD.
 * 
 * @param data The data.
 */
void LCD_send_data(uint8_t data); 

/**
 * Send string to the LCD.
 * 
 * The string length will be trimmed to the number of the LCD's columns. Also
 * multiple lines may be send at once by separating the line with a new line 
 * (\n). All lines bigger than the number of the LCD's rows will be ignore.
 * 
 * There are some modifiers which can be added on the beginning of each line:
 * <li><code>|c|</code>: will center the text</li>
 * <li><code>|l|</code>: will left-align the text (the default)</li>
 * <li><code>|r|</code>: will right-align the text</li>
 * <li><code>|d|MS|</code>: type animation with <code>MS</code> millisecond delay between each character</li>
 * 
 * @param str String to be sent.
 * @param line Line the string should be display on or begin at in case of multi-line string.
 */
void LCD_send_string(char *str, uint8_t line);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

