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

#define addr 0x07

static uint8_t rx_data[5] = {0};
//static i2cflags_t errors = 0;

static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

static void init_cols(void);
static void init_rows(void);

static void unselect_row(uint8_t row);
static void select_row(uint8_t row);

static matrix_row_t read_row(void);

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

    i2c_init_master();

    // initialize matrix state: all keys off
    for (uint8_t i=0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0;
        matrix_debouncing[i] = 0;
    }
}

uint8_t matrix_scan(void)
{
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {

        select_row(i);
        wait_us(30);  // without this wait read unstable value.

        matrix_row_t rows = read_row();
        if (matrix_debouncing[i] != rows) {
            matrix_debouncing[i] = rows;
            debouncing = DEBOUNCE;
        }
        unselect_row(i);
    }

    if (debouncing) {
        debouncing--;
    } else {
        for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
            matrix[i] = matrix_debouncing[i];
        }
    }

    i2c_scan();

    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        matrix[i] = (matrix[i] & 0x3f80) | rx_data[i];
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
static void  init_cols(void)
{
    palSetPadMode(GPIOA, 0,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOA, 1,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOA, 2,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOA, 3,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOA, 4,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOA, 5,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOA, 6,  PAL_MODE_INPUT_PULLUP);
}

static void init_rows(void)
{
    /*mark*/
    palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_PULLUP);

    palSetPadMode(GPIOB, 8, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOB, 9, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOB, 10, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOB, 11, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL);

}

/* Returns status of switches(1:on, 0:off) */
static matrix_row_t read_row(void)
{
    return (((~palReadPort(GPIOA)) & 0x7f) << 7);
}

static void select_row(uint8_t row) {
    switch (row) {
        case 0: palClearPad(GPIOB, 8);    break;
        case 1: palClearPad(GPIOB, 9);    break;
        case 2: palClearPad(GPIOB, 10);    break;
        case 3: palClearPad(GPIOB, 11);    break;
        case 4: palClearPad(GPIOB, 12);    break;
        default:break;
    }
}

static void unselect_row(uint8_t row) {
    switch (row) {
        case 0: palSetPad(GPIOB, 8);    break;
        case 1: palSetPad(GPIOB, 9);    break;
        case 2: palSetPad(GPIOB, 10);    break;
        case 3: palSetPad(GPIOB, 11);    break;
        case 4: palSetPad(GPIOB, 12);    break;
        default:break;
    }
}
