#include "LiquidCrystal_1602A.h"

uint8_t rs_lcd, en_lcd, d0_lcd, d1_lcd, d2_lcd, d3_lcd, d4_lcd, d5_lcd, d6_lcd, d7_lcd;

void lcd_begin(uint8_t rs, uint8_t en, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
	
	rs_lcd = rs;	
	en_lcd = en;
	d0_lcd = d0;	
	d1_lcd = d1;	
	d2_lcd = d2;	
	d3_lcd = d3;
	d4_lcd = d4;	
	d5_lcd = d5;	
	d6_lcd = d6;	
	d7_lcd = d7;

	PIN_MODE(rs_lcd, OUTPUT);
	PIN_MODE(en_lcd, OUTPUT);
	PIN_MODE(d0_lcd, OUTPUT);
	PIN_MODE(d1_lcd, OUTPUT);
	PIN_MODE(d2_lcd, OUTPUT);
	PIN_MODE(d3_lcd, OUTPUT);
	PIN_MODE(d4_lcd, OUTPUT);
	PIN_MODE(d5_lcd, OUTPUT);
	PIN_MODE(d6_lcd, OUTPUT);
	PIN_MODE(d7_lcd, OUTPUT);
	
	_delay_ms(15);
	lcd_command(0x38);
	_delay_ms(5);
	lcd_display_on();
	_delay_ms(8);
	lcd_command(0x06);
	_delay_ms(10);
	lcd_clear();
	_delay_ms(8);
}

void lcd_clear(void) {
	lcd_command(0x01);
	_delay_ms(2);
}

void lcd_home(void) {
	lcd_command(0x02);
	_delay_ms(2);
}

void lcd_display_on(void) {
	lcd_command(0x0C);
	_delay_ms(2);
}

void lcd_display_off(void) {
	lcd_command(0x08);
	_delay_ms(2);
}

void lcd_cursor_on(void) {
	lcd_command(0x0E);
	_delay_ms(2);
}

void lcd_cursor_off(void) {
	lcd_command(0x0C);
}

void lcd_screen_left(void) {
	lcd_command(0x18);
	_delay_ms(2);
}

void lcd_screen_right(void) {
	lcd_command(0x1C);
	_delay_ms(2);
}

void lcd_cursor_left(void) {
	lcd_command(0x10);
	_delay_ms(2);
}

void lcd_cursor_right(void) {
	lcd_command(0x14);
	_delay_ms(2);
}

void lcd_command(uint8_t command) {
	
	DIGITAL_WRITE(d0_lcd, command & (1 << 0));
	DIGITAL_WRITE(d1_lcd, command & (1 << 1));
	DIGITAL_WRITE(d2_lcd, command & (1 << 2));
	DIGITAL_WRITE(d3_lcd, command & (1 << 3));
	DIGITAL_WRITE(d4_lcd, command & (1 << 4));
	DIGITAL_WRITE(d5_lcd, command & (1 << 5));
	DIGITAL_WRITE(d6_lcd, command & (1 << 6));
	DIGITAL_WRITE(d7_lcd, command & (1 << 7));
	
	DIGITAL_WRITE(rs_lcd, LOW);
	DIGITAL_WRITE(en_lcd, HIGH);
	_delay_ms(4);
	DIGITAL_WRITE(en_lcd, LOW);
}

void lcd_write_char(char character) {

	DIGITAL_WRITE(d0_lcd, character & (1 << 0));
	DIGITAL_WRITE(d1_lcd, character & (1 << 1));
	DIGITAL_WRITE(d2_lcd, character & (1 << 2));
	DIGITAL_WRITE(d3_lcd, character & (1 << 3));
	DIGITAL_WRITE(d4_lcd, character & (1 << 4));
	DIGITAL_WRITE(d5_lcd, character & (1 << 5));
	DIGITAL_WRITE(d6_lcd, character & (1 << 6));
	DIGITAL_WRITE(d7_lcd, character & (1 << 7));
	
	DIGITAL_WRITE(rs_lcd, HIGH);
	DIGITAL_WRITE(en_lcd, HIGH);
	_delay_ms(4);
	DIGITAL_WRITE(en_lcd, LOW);
	_delay_ms(2);
}

void lcd_print(const char *characters) {
    while (*characters) {
	    lcd_write_char(*characters);
	    characters++;
    }
}

void lcd_print_float(float Variable, uint8_t NumDecimal) {
	int Variable_int = (int)(Variable);
	int Variable_dec = (int)((Variable - Variable_int) * pow(10, NumDecimal));
	
	int ConvertedLength = snprintf(NULL, 0, "%d", Variable_int) + 1;
	char* IntConverted = malloc(ConvertedLength);
	sprintf(IntConverted, "%d", Variable_int);
	lcd_print(IntConverted);
	
	ConvertedLength = snprintf(NULL, 0, "%d", Variable_dec) + 1;
	IntConverted = malloc(ConvertedLength);
	sprintf(IntConverted, "%d", Variable_dec);
	lcd_print(".");
	lcd_print(IntConverted);
}

void lcd_setcursor(int8_t row, int8_t column) {
    if (row < 1) row = 1;
    if (row > 2) row = 2;
    if (column < 1) column = 1;
    if (column > 16) column = 16;
    uint8_t position;
    if (row == 1) {
	    position = 0x00 + (column - 1);
	} 
	else {
	    position = 0x40 + (column - 1);
    }
    lcd_command(0x80 | position);
}