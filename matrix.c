/*
TinyKeyboard -- USB Keyboard firmware
Copyright (C) 2017 Guenael Jouchet

This code is based on work by Oleg Kostyuk
Copyright (C) 2013 Oleg Kostyuk <cub.uanic@gmail.com>

The large part of the driver itself is based on TMK
https://github.com/tmk/tmk_keyboard

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "matrix.h"

#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"


#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz       0x00

// I2C aliases and register addresses (see "mcp23018.md")
#define I2C_ADDR        0b0100000
#define I2C_ADDR_WRITE  ( (I2C_ADDR<<1) | I2C_WRITE )
#define I2C_ADDR_READ   ( (I2C_ADDR<<1) | I2C_READ  )
#define IODIRA          0x00            // i/o direction register
#define IODIRB          0x01

#define GPPUA           0x0C            // GPIO pull-up resistor register
#define GPPUB           0x0D
#define GPIOA           0x12            // general purpose i/o port register (write modifies OLAT)
#define GPIOB           0x13
#define OLATA           0x14            // output latch register
#define OLATB           0x15



#ifndef DEBOUNCE
#   define DEBOUNCE	5
#endif

static uint8_t debouncing = DEBOUNCE;

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];
static matrix_row_t matrix_debouncing[MATRIX_ROWS];


bool i2c_initialized = 0;

uint8_t mcp23018_status = 0x20;
//static uint8_t mcp23018_reset_loop;




uint8_t init_mcp23018(void) {
    mcp23018_status = 0x20;

    // I2C subsystem
    if (i2c_initialized == 0) {
        i2c_init();  // on pins D(1,0)
        i2c_initialized = 1;
        _delay_ms(1000);
    }

    // set pin direction
    // - unused  : input  : 1
    // - input   : input  : 1
    // - driving : output : 0
    mcp23018_status = i2c_start(I2C_ADDR_WRITE);
    if (mcp23018_status) goto out;
    mcp23018_status = i2c_write(IODIRA);
    if (mcp23018_status) goto out;
    mcp23018_status = i2c_write(0b00000000);
    if (mcp23018_status) goto out;
    mcp23018_status = i2c_write(0b00111111);
    if (mcp23018_status) goto out;
    i2c_stop();

    // set pull-up
    // - unused  : on  : 1
    // - input   : on  : 1
    // - driving : off : 0
    mcp23018_status = i2c_start(I2C_ADDR_WRITE);
    if (mcp23018_status) goto out;
    mcp23018_status = i2c_write(GPPUA);
    if (mcp23018_status) goto out;
    mcp23018_status = i2c_write(0b00000000);
    if (mcp23018_status) goto out;
    mcp23018_status = i2c_write(0b00111111);
    if (mcp23018_status) goto out;

out:
    i2c_stop();
    return mcp23018_status;
}








static void  init_cols(void) {
    // real rows, inverted cols/rows
    DDRD  &= ~(1<<3 | 1<<4 | 1<<7);
    PORTD |=  (1<<3 | 1<<4 | 1<<7);

    DDRB  &= ~(1<<5);
    PORTB |=  (1<<5);

    DDRF  &= ~(1<<5 | 1<<7);
    PORTF |=  (1<<5 | 1<<7);
}


static matrix_row_t read_cols(uint8_t row) {
    if (row >= 6) {
        if (mcp23018_status) { // if there was an error
            return 0;
        } else {
            uint8_t data = 0;
            mcp23018_status = i2c_start(I2C_ADDR_WRITE);
            if (mcp23018_status) goto out;
            mcp23018_status = i2c_write(GPIOB);
            if (mcp23018_status) goto out;
            mcp23018_status = i2c_start(I2C_ADDR_READ);
            if (mcp23018_status) goto out;
            data = i2c_readNak();
            data = ~data;
out:
            i2c_stop();
            return data;
        }
    } else {
        _delay_us(30);  // without this wait read unstable value.

        // real rows, inverted cols/rows
        return (PIND&(1<<3) ? 0 : (1<<5)) |
               (PIND&(1<<4) ? 0 : (1<<4)) |
               (PIND&(1<<7) ? 0 : (1<<3)) |
               (PINB&(1<<5) ? 0 : (1<<2)) |
               (PINF&(1<<7) ? 0 : (1<<1)) |
               (PINF&(1<<5) ? 0 : (1<<0)) ;
    }
}


inline matrix_row_t matrix_get_row(uint8_t row) {
    return matrix[row];
}


static void unselect_rows(void) {
    // unselect on mcp23018
    if (mcp23018_status) { // if there was an error
        // do nothing
    } else {
        // set all rows hi-Z : 1
        mcp23018_status = i2c_start(I2C_ADDR_WRITE);
        if (mcp23018_status) goto out;
        mcp23018_status = i2c_write(GPIOA);
        if (mcp23018_status) goto out;
        mcp23018_status = i2c_write( 0xFF );
        if (mcp23018_status) goto out;
out:
        i2c_stop();
    }

    // Hi-Z(DDR:0, PORT:0) to unselect
    DDRD  &= ~(1<<2 | 1<<5 | 1<<6);
    PORTD &= ~(1<<2 | 1<<5 | 1<<6);

    DDRB  &= ~(1<<4 | 1<<6);
    PORTB &= ~(1<<4 | 1<<6);

    DDRF  &= ~(1<<6);
    PORTF &= ~(1<<6);
}


static void select_row(uint8_t row) {
    if (row >= 6) {
        // select on mcp23018
        if (mcp23018_status) { // if there was an error
            // do nothing
        } else {
            // set active row low  : 0
            // set other rows hi-Z : 1
            mcp23018_status = i2c_start(I2C_ADDR_WRITE);
            if (mcp23018_status) goto out;
            mcp23018_status = i2c_write(GPIOA);
            if (mcp23018_status) goto out;
            mcp23018_status = i2c_write( 0xFF & ~(1<<(row-6)) );
            if (mcp23018_status) goto out;
out:
            i2c_stop();
        }
    } else {
        // select on teensy
        // Output low(DDR:1, PORT:0) to select
        switch (row) {
            case 0:
                DDRD  |=  (1<<2);
                PORTD &= ~(1<<2);
                break;
            case 1:
                DDRD  |=  (1<<5);
                PORTD &= ~(1<<5);
                break;
            case 2:
                DDRD  |=  (1<<6);
                PORTD &= ~(1<<6);
                break;
            case 3:
                DDRB  |=  (1<<4);
                PORTB &= ~(1<<4);
                break;
            case 4:
                DDRB  |=  (1<<6);
                PORTB &= ~(1<<6);
                break;
            case 5:
                DDRF  |=  (1<<6);
                PORTF &= ~(1<<6);
                break;
        }
    }
}


void matrix_init(void) {
    // Disable unused pins
    DDRB  &= ~(1<<0);
    DDRC  &= ~(1<<7 | 1<<6);
    DDRE  &= ~(1<<6);

    // Disable JTAG debugging
    MCUCR |= (1<<JTD);
    MCUCR |= (1<<JTD);

    // Init leds ON -- DEBUG
    DDRF  |= _BV(DDF0);
    PORTF |= _BV(PORTF0);
    DDRF  |= _BV(DDF1);
    PORTF |= _BV(PORTF1);
    DDRF  |= _BV(DDF4);
    PORTF |= _BV(PORTF4); 

    // Init MCP23018
    mcp23018_status = init_mcp23018();

    // Init matrix all keys off
    unselect_rows();
    init_cols();
    for (uint8_t i=0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0;
        matrix_debouncing[i] = 0;
    }
}


uint8_t matrix_scan(void) {
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        select_row(i);
        matrix_row_t cols = read_cols(i);
        if (matrix_debouncing[i] != cols) {
            matrix_debouncing[i] = cols;
            debouncing = DEBOUNCE;
        }
        unselect_rows();
    }

    if (debouncing) {
        if (--debouncing) {
            _delay_ms(1);
        } else {
            for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
                matrix[i] = matrix_debouncing[i];
            }
        }
    }
    return 1;
}

void matrix_print(void) { }