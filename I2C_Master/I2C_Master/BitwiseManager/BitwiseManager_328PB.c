#include "BitwiseManager_328PB.h"

void PIN_MODE(uint8_t pin, uint8_t mode) {
	if ((pin >= 0) && (pin < 6)) {
		if (mode == OUTPUT) {
			DDRB |= (1 << (pin % 8));
			if (pin == D12) UCSR1B &= ~(1 << RXEN1);
			if (pin == D11) UCSR1B &= ~(1 << TXEN1);
		}
		else if (mode == INPUT_PULLUP) {
			DDRB &= ~(1 << (pin % 8));
			PORTB |= (1 << (pin % 8));
		}
		else if (mode == INPUT_PULLDOWN) {
			DDRB &= ~(1 << (pin % 8));
		}
	}
	else if ((pin >= 6) && (pin < 12)) {
		if (mode == OUTPUT) {
			DDRC |= (1 << ((pin + 2) % 8));
		}
		else if (mode == INPUT_PULLUP)  {
			DDRC &= ~(1 << ((pin + 2) % 8));
			PORTC |= (1 << ((pin + 2) % 8));
		}
		else if (mode == INPUT_PULLDOWN) {
			DDRC &= ~(1 << ((pin + 2) % 8));
		}
	}
	else if ((pin >= 12) && (pin < 20)) {
		if (mode == OUTPUT) {
			DDRD |= (1 << ((pin + 4) % 8));
			if (pin == D0) UCSR0B &= ~(1 << RXEN0);
			if (pin == D1) UCSR0B &= ~(1 << TXEN0);
		}
		else if (mode == INPUT_PULLUP) {
			DDRD &= ~(1 << ((pin + 4) % 8));
			PORTD |= (1 << ((pin + 4) % 8));
		}
		else if (mode == INPUT_PULLDOWN) {
			DDRD &= ~(1 << ((pin + 4) % 8));
		}
	}
	else if ((pin == 20) || (pin == 21)) {
		if (mode == OUTPUT) {
			DDRE |= (1 << ((pin - 2) % 8));
		}
		else if (mode == INPUT_PULLUP) {
			DDRE &= ~(1 << ((pin - 2) % 8));
			PORTE |= (1 << ((pin - 2) % 8));
		}
		else if (mode == INPUT_PULLDOWN)	{
			DDRE &= ~(1 << ((pin - 2) % 8));
		}
	}
}

void DIGITAL_WRITE(uint8_t pin, uint8_t value) {
	if ((pin >= 0) && (pin < 6)) {
		if (value) {
			PORTB |= (1 << (pin % 8));
		}
		else {
			PORTB &= ~(1 << (pin % 8));
		}
	}
	else if ((pin >= 6) && (pin < 12)) {
		if (value) {
			PORTC |= (1 << ((pin + 2) % 8));
		}
		else {
			PORTC &= ~(1 << ((pin + 2) % 8));
		}
	}
	else if ((pin >= 12) && (pin < 20)) {
		if (value) {
			PORTD |= (1 << ((pin + 4) % 8));
		}
		else {
			PORTD &= ~(1 << ((pin + 4) % 8));
		}
	}
	else if ((pin == 20) || (pin == 21)) {
		if (value) {
			PORTE |= (1 << ((pin - 2) % 8));
		}
		else {
			PORTE &= ~(1 << ((pin - 2) % 8));
		}
	}
}

uint8_t DIGITAL_READ(uint8_t pin) {
	if ((pin >= 0) && (pin < 6)) {
		return (PINB >> (pin % 8)) & 1;
	}
	else if ((pin >= 6) && (pin < 12)) {
		return (PINC >> ((pin + 2) % 8)) & 1;
	}
	else if ((pin >= 12) && (pin < 20)) {
		return (PIND >> ((pin + 4) % 8)) & 1;
	}
	else if ((pin == 20) || (pin == 21)) {
		return (PINE >> ((pin - 2) % 8)) & 1;
	}
	else return INPUT_PULLUP;
}
