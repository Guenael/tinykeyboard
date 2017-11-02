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


#include "keycode.h"
#include "keymap.h"


#define KEYMAPS_SIZE    (sizeof(keymaps) / sizeof(keymaps[0]))
#define FN_ACTIONS_SIZE (sizeof(fn_actions) / sizeof(fn_actions[0]))


/* ErgoDox keymap definition macro */
#define KEYMAP(                                                 \
                                                                \
    /* left hand, spatial positions */                          \
    k00,k01,k02,k03,k04,k05,                                    \
    k10,k11,k12,k13,k14,k15,                                    \
    k20,k21,k22,k23,k24,k25,                                    \
    k30,k31,k32,k33,k34,k35,                                    \
    k40,k41,k42,k43,k44,k45,                                    \
    k50,k51,k52,k53,k54,k55,                                    \
                                                                \
    /* right hand, spatial positions */                         \
    k06,k07,k08,k09,k0A,k0B,k0C,                                \
    k16,k17,k18,k19,k1A,k1B,k1C,                                \
    k26,k27,k28,k29,k2A,k2B,k2C,                                \
    k36,k37,k38,k39,k3A,k3B,k3C,                                \
    k46,k47,k48,k49,k4A,k4B,k4C,                                \
    k56,k57,k58,k59,k5A,k5B,k5C )                               \
                                                                \
   /* matrix positions */                                       \
   {                                                            \
    { KC_##k00,KC_##k10,KC_##k20,KC_##k30,KC_##k40,KC_##k50},   \
    { KC_##k01,KC_##k11,KC_##k21,KC_##k31,KC_##k41,KC_##k51},   \
    { KC_##k02,KC_##k12,KC_##k22,KC_##k32,KC_##k42,KC_##k52},   \
    { KC_##k03,KC_##k13,KC_##k23,KC_##k33,KC_##k43,KC_##k53},   \
    { KC_##k04,KC_##k14,KC_##k24,KC_##k34,KC_##k44,KC_##k54},   \
    { KC_##k05,KC_##k15,KC_##k25,KC_##k35,KC_##k45,KC_##k55},   \
                                                                \
    { KC_##k06,KC_##k16,KC_##k26,KC_##k36,KC_##k46,KC_##k56},   \
    { KC_##k07,KC_##k17,KC_##k27,KC_##k37,KC_##k47,KC_##k57},   \
    { KC_##k08,KC_##k18,KC_##k28,KC_##k38,KC_##k48,KC_##k58},   \
    { KC_##k09,KC_##k19,KC_##k29,KC_##k39,KC_##k49,KC_##k59},   \
    { KC_##k0A,KC_##k1A,KC_##k2A,KC_##k3A,KC_##k4A,KC_##k5A},   \
    { KC_##k0B,KC_##k1B,KC_##k2B,KC_##k3B,KC_##k4B,KC_##k5B},   \
    { KC_##k0C,KC_##k1C,KC_##k2C,KC_##k3C,KC_##k4C,KC_##k5C}    \
   }

//ENT, CAPS,PGUP,PGDN,FN1, SPC, BSPC

static const uint8_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    KEYMAP(  // layer 0 : default
        // left hand
        GRV, 1,   2,   3,   4,   5,
        TAB, Q,   W,   E,   R,   T,
        LSFT,A,   S,   D,   F,   G,
        BSPC,Z,   X,   C,   V,   B,
        LCTL,LGUI,LALT,TRNS,TRNS,ENT,
        END, RALT,SPC, HOME,CAPS,DEL,

        // right hand
        6,   7,   8,   9,   0,   MINS,EQL,
        Y,   U,   I,   O,   P,   LBRC,RBRC,
        H,   J,   K,   L,   SCLN,QUOT,RSFT,
        N,   M,   COMM,DOT, SLSH,UP  ,BSLS,
        ENT, TRNS,TRNS,TRNS,LEFT,DOWN,RGHT,
        ESC, CAPS,PGUP,PGDN,FN1, SPC, BSPC
    ),

    KEYMAP(  // layer 1 : function and symbol keys
        // left hand
        ESC,F1,  F2,  F3,  F4,  F5,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        VOLU,TRNS,TRNS,TRNS,TRNS,VOLD,
        TRNS,TRNS,MUTE,TRNS,TRNS,TRNS,
        // right hand
        F6,  F7,  F8,  F9,  F10, F11, F12,
        TRNS,TRNS,TRNS,TRNS,PSCR,PAUS,NLCK,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        VOLD,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,MUTE,VOLU
    ),

    KEYMAP(  // layer 2 : keyboard functions
        // left hand
        TRNS,F1,  F2,  F3,  F4,  F5,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        // right hand
        F6,  F7,  F8,  F9,  F10, F11, F12,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS
    ),

    KEYMAP(  // layer 3: numpad
        // left hand
        TRNS,F1,  F2,  F3,  F4,  F5,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        // right hand
        F6,  F7,  F8,  F9,  F10, F11, F12,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS
    ),

};

/* id for user defined functions */
enum function_id {
    TEENSY_KEY,
};


/*
 * Fn action definition
 */
static const uint16_t PROGMEM fn_actions[] = {
    ACTION_FUNCTION(TEENSY_KEY),                    // FN0 - Teensy key
    ACTION_LAYER_MOMENTARY(1),                      // FN1 - switch to Layer1
    ACTION_LAYER_SET(2, ON_PRESS),                  // FN2 - set Layer2
    ACTION_LAYER_TOGGLE(3),                         // FN3 - toggle Layer3 aka Numpad layer
    ACTION_LAYER_SET(0, ON_PRESS),                  // FN4 - set Layer0
};


void action_function(keyrecord_t *event, uint8_t id, uint8_t opt) { }


/* translates key to keycode */
uint8_t keymap_key_to_keycode(uint8_t layer, keypos_t key) {
    if (layer < KEYMAPS_SIZE) {
        return pgm_read_byte(&keymaps[(layer)][(key.row)][(key.col)]);
    } else {
        // fall back to layer 0
        return pgm_read_byte(&keymaps[0][(key.row)][(key.col)]);
    }
}


/* translates Fn keycode to action */
action_t keymap_fn_to_action(uint8_t keycode) {
    action_t action;
    if (FN_INDEX(keycode) < FN_ACTIONS_SIZE) {
        action.code = pgm_read_word(&fn_actions[FN_INDEX(keycode)]);
    } else {
        action.code = ACTION_NO;
    }
    return action;
}
