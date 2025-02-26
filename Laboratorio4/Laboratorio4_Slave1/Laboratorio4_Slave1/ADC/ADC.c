#include "ADC.h"

void SettingADC(uint8_t ReferenceADC, uint8_t AdjustADC, uint8_t PrescalerADC, uint8_t TriggerSourceADC) {
	ADMUX = (ADMUX & 0x3F) | (ReferenceADC);				// Voltage reference selection
	ADMUX = (ADMUX & 0xDF) | (AdjustADC);					// Justification
	ADCSRA = (ADCSRA & 0xF8) | (PrescalerADC);				// Prescaler
	ADCSRB = (ADCSRB & 0xF8) | (TriggerSourceADC);			// Auto trigger source
	ADCSRA &= ~(1<<ADEN);									// Disable ADC
}

uint8_t AnalogRead(uint8_t ChannelADC) {
	DIDR0 |= (1<<ChannelADC);
	ADMUX = (ADMUX & 0xF0) | (ChannelADC);					// Conversion process (1/4)
	ADCSRA |= (1<<ADEN | 1<<ADSC);							// Conversion process (2/4)
	while (ADCSRA & (1<<ADSC));								// Conversion process (3/4)
	ADCSRA &= ~(1<<ADEN);									// Conversion process (4/4)
	DIDR0 &= ~(1<<ChannelADC);
	return ADCH;
}