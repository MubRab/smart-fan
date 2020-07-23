/*
 * File:   lcd.c
 * Author: nishar
 *
 * Created on September 10, 2019, 4:16 AM
 */


#include <xc.h>

#include "mcc_generated_files/mcc.h"
#include "i2c.h"
#include "lcd.h"

uint8_t _LCD_address;
uint8_t _LCD_cols;
uint8_t _LCD_rows;
uint8_t _LCD_backlight = LCD_BACKLIGHT;

void LCD_init(uint8_t address, uint8_t cols, uint8_t rows) {
    _LCD_address = address;
    _LCD_cols = cols;
    _LCD_rows = rows;

    LCD_send_cmd(LCD_RETURNHOME | LCD_CLEARDISPLAY);
    LCD_send_cmd(LCD_RETURNHOME | LCD_CLEARDISPLAY);
    LCD_send_cmd(LCD_RETURNHOME | LCD_CLEARDISPLAY);
    LCD_send_cmd(LCD_RETURNHOME);

    //           0b00100000        0b00001000  0b00000000    0b00000000 = 0b00101000 (0x28)
    LCD_send_cmd(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE);
    //           0b00001000           0b00000100 = 0b00001100 = (0x0C)
    LCD_send_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    //           0b00000001 (0x01)
    LCD_send_cmd(LCD_CLEARDISPLAY);
    //           0b00000100         0b00000010 = 0b00000110 (0x06)
    LCD_send_cmd(LCD_ENTRYMODESET | LCD_ENTRYLEFT);

    __delay_ms(200);
    
    LCD_send_cmd(LCD_CLEARDISPLAY);
    LCD_send_cmd(LCD_RETURNHOME);
//    LCD_backlight(false);
}

void LCD_clear(void) {
    LCD_send_cmd(LCD_CLEARDISPLAY);
    LCD_send_cmd(LCD_RETURNHOME);
    __delay_ms(200);
}

void LCD_backlight(bool on) {
    _LCD_backlight = on ? LCD_BACKLIGHT : LCD_NOBACKLIGHT;
    I2C_start();
    I2C_select(_LCD_address, I2C_MODE_WRITE);
    I2C_write(_LCD_backlight);
    I2C_stop();
}

void LCD_send_cmd(unsigned char command) {
    unsigned char nibble_lower, nibble_upper;
    nibble_lower = (command<<4) & 0xF0; //select lower nibble by moving it to the upper nibble position
    nibble_upper = command & 0xF0;      //select upper nibble
 
    I2C_start();
    I2C_select(_LCD_address, I2C_MODE_WRITE);

    I2C_write(nibble_upper | _LCD_backlight | En); // enable=1 and rs =0
    I2C_write(nibble_upper | _LCD_backlight);      // enable=0 and rs =0

    I2C_write(nibble_lower | _LCD_backlight | En); // enable =1 and rs =0
    I2C_write(nibble_lower | _LCD_backlight);      // enable=0 and rs =0
    
 
     I2C_stop();
}

void LCD_send_data(unsigned char data) {
    unsigned char nibble_lower, nibble_upper;
    nibble_lower = (data<<4) & 0xF0;  //select lower nibble by moving it to the upper nibble position
    nibble_upper = data & 0xF0;  //select upper nibble
 
    I2C_start();
    I2C_select(_LCD_address, I2C_MODE_WRITE);

    I2C_write(nibble_upper | _LCD_backlight | En | Rs); // enable=1 and rs =1 // 1101
    I2C_write(nibble_upper | _LCD_backlight | Rs);      // enable=0 and rs =1
 
    I2C_write(nibble_lower | _LCD_backlight | En | Rs); // enable=1 and rs =1
    I2C_write(nibble_lower | _LCD_backlight | Rs);      // enable=0 and rs =1
 
    I2C_stop();
}

inline void LCD_select_line(uint8_t line) {
    switch(line) {
        case 2:
            LCD_send_cmd(LCD_LINE2);
            break;
        case 3:
            LCD_send_cmd(LCD_LINE3);
            break;
        case 4:
            LCD_send_cmd(LCD_LINE4);
            break;
        default:
            LCD_send_cmd(LCD_LINE1);
            break;
    }
}

void LCD_send_string(char *str, uint8_t line) {
    while (*str && line > 0 && line <= _LCD_rows) {
        LCD_select_line(line++);
        uint8_t col=0, prefix=0;
        uint16_t i, delay = -1;
        
        if (*str == '|' && *(str + 2) == '|') {
            if (*(str + 1) == 'c' || *(str + 1) == 'r') {
                while(*(str + prefix + 3) && *(str + prefix + 3) != '\n') prefix++;
                if (prefix > _LCD_cols) prefix = _LCD_cols;
                prefix = _LCD_cols - prefix;
                if (*(str + 1) == 'c') prefix /= 2;
            }
            if (*(str + 1) == 'd') {
                i = 0;
                delay = 0;
                while(*(str + i + 3) >= '0' && *(str + i + 3) <= '9') {
                    delay = delay * 10 + (*(str + (i++) + 3) - 48);
                }
                str += i + 1;
            }
            str += 3;
        }
        
        for (i=0; i < prefix; i++) LCD_send_data(' ');
        while (*str && *str != '\n' && col++ < _LCD_cols) {
            LCD_send_data(*str++);
            if (delay == -1) __delay_us(500);
            else for(i = 0; i < delay; i++) __delay_ms(1);
        }
        if (col >= _LCD_cols) while (*str && *str != '\n') *str++;
        if (*str == '\n') *str++;
    }
}
