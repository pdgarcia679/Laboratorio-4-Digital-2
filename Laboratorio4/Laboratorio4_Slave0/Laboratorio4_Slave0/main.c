#include <avr/io.h>
#include <avr/interrupt.h>
#include "BitwiseManager/BitwiseManager_328PB.h"

#define STATUS_SR_SLA_ACK	0x60			// Own SLA+W has been received;	ACK has been returned
#define STATUS_SR_DATA_ACK	0x80			// Previously addressed with own SLA+W;	data has been received;	ACK has been returned
#define STATUS_ST_SLA_ACK	0xA8			// Own SLA+R has been	received; ACK has been returned
#define STATUS_ST_DATA_ACK	0xB8			// Data byte in TWDRn has been transmitted;	ACK has been received

ISR(TWI0_vect);
ISR(PCINT0_vect);

void INIT_TWI(uint8_t twi_address);
void DisplayLEDs(uint8_t load);

uint8_t response_data = 0;

volatile uint8_t process_data = 0;
volatile uint8_t received_data = 0;
	
volatile uint8_t counter = 0;
volatile uint8_t update_counter = 0;

int main(void)
{
    cli();
	INIT_TWI(0x42);
		
	PIN_MODE(D9, OUTPUT);
	PIN_MODE(D8, OUTPUT);
	PIN_MODE(D7, OUTPUT);
	PIN_MODE(D6, OUTPUT);
	PIN_MODE(D5, OUTPUT);
	PIN_MODE(D4, OUTPUT);
	PIN_MODE(D3, OUTPUT);
	PIN_MODE(D2, OUTPUT);
	
	
	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT2) | (1 << PCINT3);
	PIN_MODE(D10, INPUT_PULLUP);
	PIN_MODE(D11, INPUT_PULLUP);
	
	sei();
		
    while (1) 
    {
		if (process_data) {
			process_data = 0;
			switch (received_data) {
				case 0xA1:
					response_data = counter;
				break;
				default:
					response_data = '!';
				break;
			}
		}
		if (update_counter) {
			update_counter = 0;
			DisplayLEDs(counter);
		}
    }
}

void INIT_TWI(uint8_t twi_address) {
	/* (1) The device's own slave address has been received.
	*  (2) The TWEN bit enables TWI n operation and activates the TWI n interface.
	*  (3) the TWI n interrupt request will be activated for as long as the TWCRn.TWINT Flag is high. */
	TWAR0 = (twi_address << 1);
	TWCR0 = (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
}

void DisplayLEDs(uint8_t load) {
	DIGITAL_WRITE(D9, READ_BIT(load, 7));
	DIGITAL_WRITE(D8, READ_BIT(load, 6));
	DIGITAL_WRITE(D7, READ_BIT(load, 5));
	DIGITAL_WRITE(D6, READ_BIT(load, 4));
	DIGITAL_WRITE(D5, READ_BIT(load, 3));
	DIGITAL_WRITE(D4, READ_BIT(load, 2));
	DIGITAL_WRITE(D3, READ_BIT(load, 1));
	DIGITAL_WRITE(D2, READ_BIT(load, 0));
}

ISR(TWI0_vect) {
	switch (TWSR0 & 0xF8) {
		case STATUS_SR_SLA_ACK:
			TWCR0 = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
		break;
		
		case STATUS_SR_DATA_ACK:
			received_data = TWDR0;
			process_data = 1;
			TWCR0 = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
		break;
		
		case STATUS_ST_SLA_ACK:
			TWDR0 = response_data;
			TWCR0 = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
		break;
		
		case STATUS_ST_DATA_ACK:
			TWDR0 = response_data;
			TWCR0 = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
		break;
		
		default:
			TWCR0 = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
		break;
	}
}

ISR(PCINT0_vect) {
	if (!(DIGITAL_READ(D11))) {
		counter++;
		if (counter > 15) counter = 15;
		update_counter = 1;
	}
	if (!(DIGITAL_READ(D10))) {
		if (counter > 0) counter--;
		update_counter = 1;
	}
}