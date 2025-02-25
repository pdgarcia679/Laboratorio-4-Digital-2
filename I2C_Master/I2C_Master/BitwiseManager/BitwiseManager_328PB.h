#ifndef BITWISEMANAGER_328PB_H_
#define BITWISEMANAGER_328PB_H_

#include <avr/io.h>

#define SET_BIT(REG, BIT) ((REG) |= (1 << (BIT)))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(1 << (BIT)))
#define READ_BIT(REG, BIT) (((REG) >> (BIT)) & 0x01)
#define TOGGLE_BIT(REG, BIT) ((REG) ^= (1 << (BIT)))
#define COPY_BIT(REG, BIT, VAL) \
if (VAL) { SET_BIT(REG, BIT); } else { CLEAR_BIT(REG, BIT); }

#define D13		5	// (+0) % 8
#define D12		4	// (+0) % 8
#define D11		3	// (+0) % 8
#define D10		2	// (+0) % 8
#define D9		1	// (+0) % 8
#define D8		0	// (+0) % 8
#define D7		19	// (+4) % 8
#define D6		18	// (+4) % 8
#define D5		17	// (+4) % 8
#define D4		16	// (+4) % 8
#define D3		15	// (+4) % 8
#define D2		14	// (+4) % 8
#define D1		13	// (+4) % 8
#define D0		12	// (+4) % 8
#define A0		6	// (+2) % 8
#define A1		7	// (+2) % 8
#define A2		8	// (+2) % 8
#define A3		9	// (+2) % 8
#define A4		10	// (+2) % 8
#define A5		11	// (+2) % 8
#define A6		20	// (-2) % 8
#define A7		21	// (-2) % 8

#define OUTPUT			0
#define INPUT_PULLUP	1
#define INPUT_PULLDOWN	2
#define LOW			 0
#define HIGH		 1

void PIN_MODE(uint8_t pin, uint8_t mode);
void DIGITAL_WRITE(uint8_t pin, uint8_t value);
uint8_t DIGITAL_READ(uint8_t pin);

#endif /* BITWISEMANAGER_328PB_H_ */