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

#include <stdint.h>
#include <avr/io.h>
#include "print.h"
#include "debug.h"
#include "led.h"


void led_set(uint8_t usb_led)
{
/*
    // topmost - NumLock
#ifndef INVERT_NUMLOCK
    if (usb_led & (1<<USB_LED_NUM_LOCK)) {
        ergodox_right_led_1_on();
    } else {
        ergodox_right_led_1_off();
    }
#else
    if (usb_led & (1<<USB_LED_NUM_LOCK)) {
        ergodox_right_led_1_off();
    } else {
        ergodox_right_led_1_on();
    }
#endif

    // middle - CapsLock
    if (usb_led & (1<<USB_LED_CAPS_LOCK)) {
        ergodox_right_led_2_on();
    } else {
        ergodox_right_led_2_off();
    }

    // bottommost - ScrollLock
    if (usb_led & (1<<USB_LED_SCROLL_LOCK)) {
        ergodox_right_led_3_on();
    } else {
        ergodox_right_led_3_off();
    }
*/
}


// TODO

/*
void ergodox_right_led_1_on(void)    {
    DDRB |=  (1<<5);
    PORTB |=  (1<<5);
}
void ergodox_right_led_2_on(void)    {
    DDRB |=  (1<<6);
    PORTB |=  (1<<6);
}
void ergodox_right_led_3_on(void)    {
    DDRB |=  (1<<7);
    PORTB |=  (1<<7);
}
*/