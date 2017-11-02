/*
TinyKeyboard -- USB Keyboard firmware
Copyright (C) 2017 Guenael Jouchet

This code is based on work by Oleg Kostyuk
Copyright (C) 2013 Oleg Kostyuk <cub.uanic@gmail.com>

The large part of the driver itself is based on TMK
https://github.com/tmk/tmk_keyboard

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
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

#define USB_LED_NUM_LOCK                0
#define USB_LED_CAPS_LOCK               1
#define USB_LED_SCROLL_LOCK             2
#define USB_LED_COMPOSE                 3
#define USB_LED_KANA                    4


#ifndef DEBOUNCE
#   define DEBOUNCE	2
#endif

static uint8_t debouncing = DEBOUNCE;

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];
static matrix_row_t matrix_debouncing[MATRIX_ROWS];


uint8_t i2c_initialized = 0;
uint8_t mcp23018_status;


uint8_t init_mcp23018(void) {
    // I2C subsystem
    if (i2c_initialized == 0) {
        i2c_init();  // on pins D(1,0)
        i2c_initialized = 1;
        _delay_ms(100);
    }

    // set pin direction
    mcp23018_status = i2c_start(I2C_ADDR_WRITE);
    if (mcp23018_status == 0)
        mcp23018_status = i2c_write(IODIRA);
    if (mcp23018_status == 0)
        mcp23018_status = i2c_write(0b00000000);
    if (mcp23018_status == 0)
        mcp23018_status = i2c_write(0b00111111);
    if (mcp23018_status == 0)
        i2c_stop();

    // set pull-up
    mcp23018_status = i2c_start(I2C_ADDR_WRITE);
    if (mcp23018_status == 0)
        mcp23018_status = i2c_write(GPPUA);
    if (mcp23018_status == 0)
        mcp23018_status = i2c_write(0b00000000);
    if (mcp23018_status == 0)
        mcp23018_status = i2c_write(0b00111111);
    if (mcp23018_status == 0)
        i2c_stop();
    
    return mcp23018_status;
}


static void  init_cols(void) {
    // FIX real rows, inverted cols/rows
    DDRD  &= ~(_BV(DDD3)   | _BV(DDD4)   | _BV(DDD7));
    PORTD |=  (_BV(PORTD3) | _BV(PORTD4) | _BV(PORTD7));

    DDRB  &= ~_BV(DDB5);
    PORTB |=  _BV(PORTB5);

    DDRF  &= ~(_BV(DDF5)   | _BV(DDF7));
    PORTF |=  (_BV(PORTF5) | _BV(PORTF7));
}


static matrix_row_t read_cols(uint8_t row) {
    if (row >= 6) {
        /* Remote side (MCP23018) */
        uint8_t data = 0;

        if (mcp23018_status == 0)
            mcp23018_status = i2c_start(I2C_ADDR_WRITE);

        if (mcp23018_status == 0)
            mcp23018_status = i2c_write(GPIOB);

        if (mcp23018_status == 0)
            mcp23018_status = i2c_start(I2C_ADDR_READ);

        if (mcp23018_status == 0) {
            data = ~i2c_readNak();
            i2c_stop();
        }

        return data;
    } else {
        /* CTRL side */
        _delay_us(30);  // without this wait read unstable value.

        // Hardware mapping
        // FIX real rows, inverted cols/rows
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
    /* Remote side (MCP23018) -- Unselect any with Hi-Z */
    if (mcp23018_status == 0)
        mcp23018_status = i2c_start(I2C_ADDR_WRITE);

    if (mcp23018_status == 0)
        mcp23018_status = i2c_write(GPIOA);

    if (mcp23018_status == 0)
        mcp23018_status = i2c_write(0xFF);

    if (mcp23018_status == 0)
        i2c_stop();

    /* CTRL side -- Hi-Z to unselect */
    DDRD  &= ~(_BV(DDD2)   | _BV(DDD5)   | _BV(DDD6));
    PORTD &= ~(_BV(PORTD2) | _BV(PORTD5) | _BV(PORTD6));

    DDRB  &= ~(_BV(DDB4)   | _BV(DDB6));
    PORTB &= ~(_BV(PORTB4) | _BV(PORTB6));

    DDRF  &= ~_BV(DDF6);
    PORTF &= ~_BV(PORTF6);
}


static void select_row(uint8_t row) {
    if (row >= 6) {
        /* Remote side (MCP23018) */
        if (mcp23018_status == 0)
            mcp23018_status = i2c_start(I2C_ADDR_WRITE);

        if (mcp23018_status == 0)
            mcp23018_status = i2c_write(GPIOA);

        if (mcp23018_status == 0)
            mcp23018_status = i2c_write( 0xFF & ~(1<<(row-6)) );

        if (mcp23018_status == 0)
            i2c_stop();
    } else {
        /* CTRL side -- Output low to select */
        switch (row) {
            case 0:
                DDRD  |=  _BV(DDD2);
                PORTD &= ~_BV(PORTD2);
                break;
            case 1:
                DDRD  |=  _BV(DDD5);
                PORTD &= ~_BV(PORTD5);
                break;
            case 2:
                DDRD  |=  _BV(DDD6);
                PORTD &= ~_BV(PORTD6);
                break;
            case 3:
                DDRB  |=  _BV(DDB4);
                PORTB &= ~_BV(PORTB4);
                break;
            case 4:
                DDRB  |=  _BV(DDB6);
                PORTB &= ~_BV(PORTB6);
                break;
            case 5:
                DDRF  |=  _BV(DDB6);
                PORTF &= ~_BV(PORTF6);
                break;
        }
    }
}


void matrix_init(void) {
    /* Disable JTAG debugging */
    MCUCR |= (1<<JTD);
    MCUCR |= (1<<JTD);

    /* Disable unused pins */
    DDRB  &= ~_BV(DDB0);
    DDRC  &= ~(_BV(DDC7) | _BV(DDC6));
    DDRE  &= ~_BV(DDE6);

    /* Init leds */
    DDRF  |= _BV(DDF0);
    DDRF  |= _BV(DDF1);
    DDRF  |= _BV(DDF4);

    /* Init MCP23018 */
    mcp23018_status = init_mcp23018();

    /* Init matrix all keys off */
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

void led_set(uint8_t usb_led) {
    /* CapsLock */
    if (usb_led & (1<<USB_LED_CAPS_LOCK)) {
        PORTF |= _BV(PORTF4);
    } else {
        PORTF &= ~_BV(PORTF4);
    }

    /* NumLock */
    if (usb_led & (1<<USB_LED_NUM_LOCK)) {
        PORTF |= _BV(PORTF0);
    } else {
        PORTF &= ~_BV(PORTF0);
    }

    /* ScrollLock */
    if (usb_led & (1<<USB_LED_SCROLL_LOCK)) {
        PORTF |= _BV(PORTF1); 
    } else {
        PORTF &= ~_BV(PORTF1); 
    }
}

void matrix_print(void) { }