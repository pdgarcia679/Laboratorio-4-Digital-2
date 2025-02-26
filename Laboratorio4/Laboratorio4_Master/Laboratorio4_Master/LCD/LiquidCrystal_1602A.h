#ifndef LIQUIDCRYSTAL_1602A_H_
#define LIQUIDCRYSTAL_1602A_H_

#ifndef	F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>
#include "BitwiseManager/BitwiseManager_328PB.h"

extern	uint8_t rs_lcd, en_lcd, d0_lcd, d1_lcd, d2_lcd, d3_lcd, d4_lcd, d5_lcd, d6_lcd, d7_lcd;

void lcd_begin(uint8_t rs, uint8_t en, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);

void lcd_clear(void);

void lcd_home(void);

void lcd_display_on(void);

void lcd_display_off(void);

void lcd_cursor_on(void);

void lcd_cursor_off(void);

void lcd_screen_left(void);

void lcd_screen_right(void);

void lcd_cursor_left(void);

void lcd_cursor_right(void);

void lcd_command(uint8_t command);

void lcd_write_char(char character);

void lcd_print(const char *characters);

void lcd_print_float(float Variable, uint8_t NumDecimal);

void lcd_setcursor(int8_t row, int8_t column);

#endif /* LIQUIDCRYSTAL_1602A_H_ */