#include <avr/io.h>
#include <avr/interrupt.h>
#include "BitwiseManager/BitwiseManager_328PB.h"
#include "ADC/ADC.h"

#define STATUS_SR_SLA_ACK	0x60			// Own SLA+W has been received;	ACK has been returned
#define STATUS_SR_DATA_ACK	0x80			// Previously addressed with own SLA+W;	data has been received;	ACK has been returned
#define STATUS_ST_SLA_ACK	0xA8			// Own SLA+R has been	received; ACK has been returned
#define STATUS_ST_DATA_ACK	0xB8			// Data byte in TWDRn has been transmitted;	ACK has been received

ISR(TWI0_vect);

void INIT_TWI(uint8_t twi_address);
void DisplayLEDs(uint8_t load);

uint8_t response_data = 0;

volatile uint8_t process_data = 0;
volatile uint8_t received_data = 0;
	
uint8_t actual_sensor = 0;
uint8_t prev_sensor = 0;
const uint8_t sensibility_mask = 0b11111110;

int main(void)
{
    cli();
	INIT_TWI(0x36);
	SettingADC(AVcc, LeftJustified, ADC_Prescaler128, FreeRunning);
	sei();
	
	PIN_MODE(D9, OUTPUT);
	PIN_MODE(D8, OUTPUT);
	PIN_MODE(D7, OUTPUT);
	PIN_MODE(D6, OUTPUT);
	PIN_MODE(D5, OUTPUT);
	PIN_MODE(D4, OUTPUT);
	PIN_MODE(D3, OUTPUT);
	PIN_MODE(D2, OUTPUT);
			
    while (1) 
    {
		actual_sensor = AnalogRead(ChannelADC7);
		if ( (actual_sensor & sensibility_mask) != (prev_sensor & sensibility_mask)) {
			prev_sensor = actual_sensor;
			DisplayLEDs(actual_sensor);
		}
		if (process_data) {
			process_data = 0;
			switch (received_data) {
				case 0xA1:
					response_data = prev_sensor;
				break;
				default:
					response_data = 0xFF;
				break;
			}
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
	DIGITAL_WRITE(D2, READ_BIT(load, 7));
	DIGITAL_WRITE(D3, READ_BIT(load, 6));
	DIGITAL_WRITE(D4, READ_BIT(load, 5));
	DIGITAL_WRITE(D5, READ_BIT(load, 4));
	DIGITAL_WRITE(D6, READ_BIT(load, 3));
	DIGITAL_WRITE(D7, READ_BIT(load, 2));
	DIGITAL_WRITE(D8, READ_BIT(load, 1));
	DIGITAL_WRITE(D9, READ_BIT(load, 0));
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