/*
 * File:   i2c.c
 * Author: nishar
 *
 * Created on September 10, 2019, 4:13 AM
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "mcc_generated_files/mcc.h"
#include "i2c.h"

inline void I2C_wait(void) {
    // wait for start bit to clear in SSPSTAT and bits 0 to 4 in SSPCON2
    while ((I2C_SSPSTAT & 0x04) || (I2C_SSPCON2 & 0x1F));
}

void I2C_start(void) {
    I2C_wait();
    I2C_SSPCON2 |= 0x01; // SEN=1 -> initiate the START condition on SDA and SCL pins
}

void I2C_repeated_start(void) {
    I2C_wait();
    I2C_SSPCON2 |= 0x02; // RSEN=1  -> initiate REPEATED START condition on SDA and SCL pins
}

void I2C_stop(void) {
    I2C_wait();
    I2C_SSPCON2 |= 0x04; // PEN=1 -> initiate the STOP condition on SDA and SCL pins
}

void I2C_select(uint8_t address, bool mode) {
    I2C_write(address << 1 | mode);
}

uint8_t I2C_read(uint8_t ack) {
    uint8_t temp;
    I2C_wait();
    RCEN1 = 1;      // enable receive mode for I2c 
    I2C_wait();
    temp = SSP1BUF; // load data from Buffer to the temp
    I2C_wait();
    ACKDT1 = (ack); // 0-- not ACK , 1-- ACK
    ACKEN1 = 1;     // Send Acknowledgement
    return temp;
}

void I2C_write(uint8_t data) {
    I2C_wait();
    SSP1BUF = data;  // load data into SSPBUF register
}

void I2C_send(uint8_t address, uint8_t data) {
    I2C_start();
    I2C_select(address, I2C_MODE_WRITE);
    I2C_write(data);
    I2C_stop();
}

void I2C_sendN(uint8_t address, uint8_t n, ...) {
    uint8_t i;
    va_list argp;
    va_start(argp, n);
    I2C_start();
    I2C_select(address, I2C_MODE_WRITE);
    for (i=0; i < n; i++) {
        I2C_write(va_arg(argp, uint8_t));
    }
    I2C_stop();
    va_end(argp);
}

uint8_t I2C_receive(uint8_t address, uint8_t reg) {
    I2C_send(address, reg);

    I2C_start();
    I2C_select(address, I2C_MODE_READ);
    uint8_t data = I2C_read(I2C_ACK);
    I2C_stop();
    return data;
}
