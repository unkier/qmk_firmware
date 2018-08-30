/*
Copyright 2012 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "hal.h"

/*
 * scan matrix
 */
#include "print.h"
#include "debug.h"
#include "util.h"
#include "matrix.h"
#include "wait.h"

#ifndef DEBOUNCE
#   define DEBOUNCE 5
#endif
static uint8_t debouncing = DEBOUNCE;

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];
static matrix_row_t matrix_debouncing[MATRIX_ROWS];
static uint8_t scan_start , scan_end = 0;
static uint8_t is_master = 0;

#define addr 0x07

static uint8_t rx_data[5] = {0};
//static i2cflags_t errors = 0;

static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};




//static matrix_row_t read_cols(void);
static void init_cols(void);
static void init_rows(void);

static void unselect_row(uint8_t row);
static void select_row(uint8_t row);


static matrix_row_t read_cols(void);

inline
uint8_t matrix_rows(void)
{
    return MATRIX_ROWS;
}

inline
uint8_t matrix_cols(void)
{
    return MATRIX_COLS;
}

/* generic STM32F103C8T6 board */
#ifdef BOARD_GENERIC_STM32_F103
#define LED_ON()    do {/* palSetPad(GPIOB, 12)*/ ;} while (0)
#define LED_OFF()   do { /*palClearPad(GPIOB, 12);*/ } while (0)
#define LED_TGL()   do { /*palTogglePad(GPIOB, 12);*/ } while (0)
#endif

/* Maple Mini */
#ifdef BOARD_MAPLEMINI_STM32_F103
#define LED_ON()    do { palSetPad(GPIOB, 1) ;} while (0)
#define LED_OFF()   do { palClearPad(GPIOB, 1); } while (0)
#define LED_TGL()   do { palTogglePad(GPIOB, 1); } while (0)
#endif

void i2c_init_master(void) {
    i2cStart(&I2CD1, &i2cfg1);
}


void i2c_scan(void) {
    //msg_t status = MSG_OK;
    systime_t tmo = MS2ST(10);

    i2cAcquireBus(&I2CD1);
    /*status =*/ i2cMasterReceiveTimeout(&I2CD1, addr, rx_data, 5, tmo);
    i2cReleaseBus(&I2CD1);

    /*if (status == MSG_RESET){
        errors = i2cGetErrors(&I2CD1);
        osalDbgCheck(I2C_ACK_FAILURE == errors);
    }*/
}

void matrix_init(void)
{
    // initialize row and col
    init_cols();
    init_rows();

    palSetPadMode(GPIOB, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);   /* SCL */
    palSetPadMode(GPIOB, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);   /* SDA */

    //if (palReadPort(GPIOB) & 0x01) {
    scan_start = 7;
    scan_end = MATRIX_COLS;
    is_master = 1;

    /*} else {
        scan_start = 0;
        scan_end = 7;
    }*/

    i2c_init_master();

    // initialize matrix state: all keys off
    for (uint8_t i=0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0;
        matrix_debouncing[i] = 0;
    }

    //debug

    /*debug_matrix = true;
    LED_ON();
    wait_ms(500);
    LED_OFF();*/



}

uint8_t matrix_scan(void)
{
    bool matrix_changed = false;



    for (uint8_t i = scan_start; i < scan_end; i++) {

        select_row(i);
        wait_us(30);  // without this wait read unstable value.

        matrix_row_t cols = read_cols();


        matrix_changed = false;
        for (uint8_t j = 0; j < MATRIX_ROWS; ++j) {
            matrix_row_t last_row_value = matrix_debouncing[j];

            if ((cols >> j) & 0x01) {
                matrix_debouncing[j] |= (1 << i);
            } else {
                matrix_debouncing[j] &= ~(1 << i);
            }

            if (matrix_debouncing[j] != last_row_value) {
                matrix_changed = true;
            }

        }

        unselect_row(i);


        if (matrix_changed) {
            debouncing = DEBOUNCE;
        }


    }

    if (debouncing) {
        if (--debouncing) {
            wait_ms(1);
        } else {
            for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
                matrix[i] = matrix_debouncing[i];
            }
        }
    }

    i2c_scan();

    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        matrix[i] = (matrix[i] & 0xff00) | rx_data[i];
        rx_data[i] = 0;
    }

    return 1;
}

inline
bool matrix_is_on(uint8_t row, uint8_t col)
{
    return (matrix[row] & ((matrix_row_t)1<<col));
}

inline
matrix_row_t matrix_get_row(uint8_t row)
{
    return matrix[row];
}

void matrix_print(void)
{
    print("\nr/c 0123456789ABCDEF\n");
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        phex(row); print(": ");
        pbin_reverse16(matrix_get_row(row));
        print("\n");
    }
}

/* Column pin configuration
 */
static void  init_rows(void)
{
    palSetPadMode(GPIOA, 0,  PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOA, 1,  PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOA, 2,  PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOA, 3,  PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOA, 4,  PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOA, 5,  PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOA, 6,  PAL_MODE_OUTPUT_PUSHPULL);
}


static void init_cols(void)
{
    /*mark*/
    palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_PULLUP);

    palSetPadMode(GPIOB, 8, PAL_MODE_INPUT_PULLDOWN);
    palSetPadMode(GPIOB, 9, PAL_MODE_INPUT_PULLDOWN);
    palSetPadMode(GPIOB, 10, PAL_MODE_INPUT_PULLDOWN);
    palSetPadMode(GPIOB, 11, PAL_MODE_INPUT_PULLDOWN);
    palSetPadMode(GPIOB, 12, PAL_MODE_INPUT_PULLDOWN);

}

/* Returns status of switches(1:on, 0:off) */
static matrix_row_t read_cols(void)
{
    return (palReadPort(GPIOB) >> 8) & 0x1f;
}


static void select_row(uint8_t col) {
    switch (col) {
        case 0: palSetPad(GPIOA, 0);    break;
        case 1: palSetPad(GPIOA, 1);    break;
        case 2: palSetPad(GPIOA, 2);    break;
        case 3: palSetPad(GPIOA, 3);    break;
        case 4: palSetPad(GPIOA, 4);    break;
        case 5: palSetPad(GPIOA, 5);    break;
        case 6: palSetPad(GPIOA, 6);    break;

        case 7: palSetPad(GPIOA, 0);    break;
        case 8: palSetPad(GPIOA, 1);    break;
        case 9: palSetPad(GPIOA, 2);    break;
        case 10: palSetPad(GPIOA, 3);    break;
        case 11: palSetPad(GPIOA, 4);    break;
        case 12: palSetPad(GPIOA, 5);    break;
        case 13: palSetPad(GPIOA, 6);    break;
        default:break;
    }
}

static void unselect_row(uint8_t col) {
    switch (col) {
        case 0: palClearPad(GPIOA, 0);    break;
        case 1: palClearPad(GPIOA, 1);    break;
        case 2: palClearPad(GPIOA, 2);    break;
        case 3: palClearPad(GPIOA, 3);    break;
        case 4: palClearPad(GPIOA, 4);    break;
        case 5: palClearPad(GPIOA, 5);    break;
        case 6: palClearPad(GPIOA, 6);    break;

        case 7: palClearPad(GPIOA, 0);    break;
        case 8: palClearPad(GPIOA, 1);    break;
        case 9: palClearPad(GPIOA, 2);    break;
        case 10: palClearPad(GPIOA, 3);    break;
        case 11: palClearPad(GPIOA, 4);    break;
        case 12: palClearPad(GPIOA, 5);    break;
        case 13: palClearPad(GPIOA, 6);    break;
    }
}
