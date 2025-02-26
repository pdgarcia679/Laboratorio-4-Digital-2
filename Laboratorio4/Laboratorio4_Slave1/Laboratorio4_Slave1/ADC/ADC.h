#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <stdint.h>

#define ChannelADC0			0
#define ChannelADC1			(1<<MUX0)
#define ChannelADC2			(1<<MUX1)
#define ChannelADC3			(1<<MUX1 | 1<<MUX0)
#define ChannelADC4			(1<<MUX2)
#define ChannelADC5			(1<<MUX2 | 1<<MUX0)
#define ChannelADC6			(1<<MUX2 | 1<<MUX1)
#define ChannelADC7			(1<<MUX2 | 1<<MUX1 | 1<<MUX0)

#define ADC_Prescaler2A		0
#define ADC_Prescaler2B		(1<<ADPS0)
#define ADC_Prescaler4		(1<<ADPS1)
#define ADC_Prescaler8		(1<<ADPS1 | 1<<ADPS0)
#define ADC_Prescaler16		(1<<ADPS2)
#define ADC_Prescaler32		(1<<ADPS2 | 1<<ADPS0)
#define ADC_Prescaler64		(1<<ADPS2 | 1<<ADPS1)
#define ADC_Prescaler128	(1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0)

#define ADC_Resolution		1023
#define Voltage_AVcc		5.0

#define AREF				0
#define AVcc				(1<<REFS0)
#define Internal			(1<<REFS1 | 1<<REFS0)

#define RightJustified		0
#define LeftJustified		(1<<ADLAR)

#define FreeRunning			0
#define AnalogComparator	(1<<ADTS0)
#define ExternalInterrupt	(1<<ADTS1)
#define Timer0Compare		(1<<ADTS1 | 1<<ADTS0)
#define Timer0Overflow		(1<<ADTS2)
#define Timer1Compare		(1<<ADTS2 | 1<<ADTS0)
#define Timer1Overflow		(1<<ADTS2 | 1<<ADTS1)
#define Timer1Capture		(1<<ADTS2 | 1<<ADTS1 | 1<<ADTS0)

void SettingADC(uint8_t ReferenceADC, uint8_t AdjustADC, uint8_t PrescalerADC, uint8_t TriggerSourceADC);
uint8_t AnalogRead(uint8_t ChannelADC);

#endif