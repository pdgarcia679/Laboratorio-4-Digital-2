#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include "BitwiseManager/BitwiseManager_328PB.h"

/* Master wants to send data to a slave */
// (1) Master-transmitter sends a START condition and addresses the slave-receiver
// (2) Master-transmitter sends data to slave-receiver
// (3) Master-transmitter terminates the transfer with a STOP condition

/* Master wants to read data from a slave */
// (1) Master-receiver sends a START condition and addresses the slave-transmitter
// (2) Master-receiver sends the requested register to read to slave-transmitter
// (3) Master-receiver receives data from the slave-transmitter
// (4) Master-receiver terminates the transfer with a STOP condition

#define STATUS_START		0x08
#define STATUS_MT_SLA_ACK	0x18
#define STATUS_MT_DATA_ACK	0x28
#define STATUS_MR_SLA_ACK	0x40
#define STATUS_MR_DATA_ACK	0x50
#define STATUS_MR_DATA_NACK 0x58
#define STATUS_REP_START	0x10
/*
typedef enum {
	STATUS_START = 0x08,				// A START condition has been	transmitted
	STATUS_REP_START = 0x10,			// A repeated START condition	has been transmitted
	STATUS_MT_SLA_ACK = 0x18,			// SLA+W has been transmitted; ACK has been received
	STATUS_MT_SLA_NACK = 0x20,			// SLA+W has been transmitted; NOT ACK has been received
	STATUS_MT_DATA_ACK = 0x28,			// Data byte has been	transmitted; ACK has been received
	STATUS_MT_DATA_NACK = 0x30,			// Data byte has been	transmitted; NOT ACK has been received
	STATUS_MT_ARB_LOST = 0x38,			// Arbitration lost in SLA+R/W or data bytes

	STATUS_MR_SLA_ACK = 0x40,			// SLA+R has been transmitted; ACK has been received
	STATUS_MR_SLA_NACK = 0x48,			// SLA+R has been transmitted; NOT ACK has been received
	STATUS_MR_DATA_ACK = 0x50,			// Data byte has been received; ACK has been returned
	STATUS_MR_DATA_NACK = 0x58,			// Data byte has been received; NOT ACK has been returned

	STATUS_SR_SLA_ACK = 0x60,			// Own SLA+W has been received;	ACK has been returned
	STATUS_SR_ARB_LOST_SLA_ACK = 0x68,	// Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
	STATUS_SR_DATA_ACK = 0x80,			// Previously addressed with own SLA+W;	data has been received;	ACK has been returned
	STATUS_SR_DATA_NACK = 0x88,			// Previously addressed with own SLA+W;	data has been received;	NOT ACK has been returned
	STATUS_SR_STOP = 0xA0,				// A STOP condition or repeated	START condition has been received while still addressed	as Slave

	STATUS_ST_SLA_ACK = 0xA8,			// Own SLA+R has been	received; ACK has been returned
	STATUS_ST_ARB_LOST_SLA_ACK = 0xB0,	// Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
	STATUS_ST_DATA_ACK = 0xB8,			// Data byte in TWDRn has been transmitted;	ACK has been received
	STATUS_ST_DATA_NACK = 0xC0,			// Data byte in TWDRn has been transmitted; NOT ACK has been received
	STATUS_ST_LAST_DATA = 0xC8,			// Last data byte in TWDRn has been transmitted (TWEA =	“0”); ACK has been received

	STATUS_NO_INFO = 0xF8,				// No relevant state information available; TWINT = “0”
	STATUS_BUS_ERROR = 0x00				// Bus error due to an illegal START or STOP condition
};
*/
uint8_t TWI_WriteData(uint8_t twi_address, uint8_t twi_load);
uint8_t TWI_ReadData(uint8_t twi_address, uint8_t data_address, uint8_t* data_pointer);

void Init_TWI();
void TWI_ACK();
void TWI_START();
void TWI_STOP();
// uint8_t TWI_STATUS();

uint8_t received_data;

int main(void)
{
    cli();
	Init_TWI();
	sei();
	
	PIN_MODE(D9, OUTPUT);
	PIN_MODE(D8, OUTPUT);
	PIN_MODE(D7, OUTPUT);
	PIN_MODE(D6, OUTPUT);
	PIN_MODE(D5, OUTPUT);
	PIN_MODE(D4, OUTPUT);
	PIN_MODE(D3, OUTPUT);
	PIN_MODE(D2, OUTPUT);
	
	_delay_ms(10);
						
	if (TWI_WriteData(0x36, 0b10101010)) {
		
	}
	_delay_ms(10);
	if (TWI_ReadData(0x36, 0xA1, &received_data)) {
		
	}
	
	DIGITAL_WRITE(D9, READ_BIT(received_data, 7));
	DIGITAL_WRITE(D8, READ_BIT(received_data, 6));
	DIGITAL_WRITE(D7, READ_BIT(received_data, 5));
	DIGITAL_WRITE(D6, READ_BIT(received_data, 4));
	DIGITAL_WRITE(D5, READ_BIT(received_data, 3));
	DIGITAL_WRITE(D4, READ_BIT(received_data, 2));
	DIGITAL_WRITE(D3, READ_BIT(received_data, 1));
	DIGITAL_WRITE(D2, READ_BIT(received_data, 0));
	
    while (1) 
    {	
		/* Needs more information to define the specific trajectory and frame (big function) */
		/* User can use different functions and each pass the specific parameters to the big function */
	}
}

uint8_t TWI_WriteData(uint8_t twi_address, uint8_t twi_load) {
	TWI_START();
	while(!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != STATUS_START)	{
		TWI_STOP();
		return 1;
	}
	
	TWDR0 = (twi_address << 1) | 0;
	TWCR0 = (1<<TWEN) | (1<<TWINT);
	while(!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != STATUS_MT_SLA_ACK)	{
		TWI_STOP();
		return 1;
	}
	
	TWDR0 = twi_load;
	TWCR0 = (1<<TWEN) | (1<<TWINT);
	while(!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != STATUS_MT_DATA_ACK)	{
		TWI_STOP();
		return 1;
	}
	
	TWI_STOP();
	return 0;
}

uint8_t TWI_ReadData(uint8_t twi_address, uint8_t data_address, uint8_t* data_pointer) {
	TWI_START();
	while (!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != STATUS_START)	{
		TWI_STOP();
		return 1;
	}
	
	TWDR0 = (twi_address << 1) | 0;
	TWCR0 = (1<<TWEN) | (1<<TWINT);
	while(!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != STATUS_MT_SLA_ACK)	{
		TWI_STOP();
		return 1;
	}
	
	TWDR0 = data_address;
	TWCR0 = (1<<TWEN) | (1<<TWINT);
	while(!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != STATUS_MT_DATA_ACK)	{
		TWI_STOP();
		return 1;
	}
	
	TWI_START();
	while (!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != STATUS_REP_START)	{
		TWI_STOP();
		return 1;
	}
	
	TWDR0 = (twi_address << 1) | 1;
	TWCR0 = (1<<TWEN) | (1<<TWINT);
	while(!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != STATUS_MR_SLA_ACK) {
		TWI_STOP();
		return 1;
	}
		
	TWCR0 = (1<<TWEN) | (1<<TWINT);
	while(!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != STATUS_MR_DATA_NACK)	{	// Other does not verify this
		TWI_STOP();
		return 1;
	}
	*data_pointer = TWDR0;
	
	TWI_STOP();
	return 0;
}

void Init_TWI() {
	TWSR0 &= ~(1<<TWPS0);
	TWSR0 &= ~(1<<TWPS1);
	TWBR0 = 72;
	TWCR0 = (1<<TWEN);
}

void TWI_START() {
	/* (1) The TWI hardware checks if the bus is available, and generates a START condition on the bus if it is free.
	*  (2) The TWEN bit enables TWI operation and activates the TWI interface.
	*  (3) This bit is set by hardware when the TWI has finished its current job and expects application software response. */
	TWCR0 = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT);	// (1) | (2) | (3)
}

void TWI_STOP() {
	/* (1) Writing the TWSTO bit to one in Master mode will generate a STOP condition on the 2-wire Serial Bus TWI.
	*  (2) The TWEN bit enables TWI operation and activates the TWI interface.
	*  (3) This bit is set by hardware when the TWI has finished its current job and expects application software response. */
	TWCR0 |= (1<<TWSTO) | (1<<TWEN) | (1<<TWINT);	// (1) | (2) | (3)
	while (TWCR0 & (1<<TWSTO));
}
/*
uint8_t TWI_STATUS() { // Needs to know: twi_mode, twi_load
	// (1) TWSTA must be cleared by	software when the START condition has been transmitted.
	// (2) The TWEN bit enables TWI operation and activates the TWI interface.
	// (3) The TWINT Flag must be cleared by software by writing a logic one to it.	
	// (4) Writing the TWSTO bit to one in Master mode will generate a STOP condition on the 2-wire Serial Bus TWI.
	twi_status = TWSR0 & 0xF8;
	switch (twi_status) {
		case STATUS_START:
			TWDR0 = (twi_address << 1) | twi_mode;
			TWCR0 = (1<<TWEN) | (1<<TWINT);
		break;
		case STATUS_REP_START:
			TWDR0 = (twi_address << 1) | twi_mode;
			TWCR0 = (1<<TWEN) | (1<<TWINT);		// ~(1) | (2) | (3)
		break;
		
		case STATUS_MT_SLA_ACK:
			TWDR0 = twi_load;
			TWCR0 = (1<<TWEN) | (1<<TWINT);		// ~(1) | (2) | (3)
		break;
		
		case STATUS_MT_SLA_NACK:
			TWI_STOP();
		break;
		
		case STATUS_MT_DATA_ACK:
			TWCR0 = (1<<TWEN) | (1<<TWINT);		// ~(1) | (2) | (3)
		break;
		
		case STATUS_MT_DATA_NACK:
			TWI_STOP();
		break;
	}
}
*/