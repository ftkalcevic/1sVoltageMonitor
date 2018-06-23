/*
 * 1sVoltageMonitor.c
 *
 * Created: 22/06/2018 8:06:59 PM
 * Author : Frank Tkalcevic
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <util/crc16.h>
#include <avr/interrupt.h>


#define LED_PIN		PB0
#define BUZZER_PIN	PB4
#define ADC_PIN		PB3
#define SWITCH_PIN	PB1

#define SETUP_SAMPLES 16

static volatile uint8_t ms;

struct EEPROMData
{
	int8_t crc;
	uint16_t low_voltage;
};

uint8_t calcCRC(uint16_t value)
{
	uint8_t crc = 0x72;
	crc = _crc_ibutton_update(crc, value & 0xFF);
	crc = _crc_ibutton_update(crc, value >> 8);
	return crc;
}

int main(void)
{
	if ( MCUSR & ~_BV(WDRF) )
	{
		// Watchdog reset (wake from sleep)
	}

	// Outputs
	DDRB |= _BV(LED_PIN) | _BV(BUZZER_PIN);
	// Pullup on switch input
	PORTB |= _BV(SWITCH_PIN);

	// Set up ADC
	ADMUX = _BV(REFS2) | _BV(REFS1) |	// Internal 2.56v reference
			0 |							// Right adjust result
			_BV(MUX1) | _BV(MUX0);		// ADC3(PB3)
	ADCSRA = _BV(ADEN) |				// ADC Enable
			0 |							// manual trigger
			0 |							// No interrupts
			_BV(ADPS1) | _BV(ADPS0);	// Prescale x8
	ADCSRB = 0;							
	DIDR0 = _BV(ADC3D);
	_delay_ms(1);

	// Setup tranducer timer1
	TCCR1 = _BV(CTC1) |					// Clear counter on OCR1C match
			0;							// clock stopped
	GTCCR = _BV(PWM1B) |				// PWM mode
			_BV(COM1B0);				// toggle oc1b
	OCR1B = 111;
	OCR1C = 222;


	// throw the first value away
	ADCSRA |= _BV(ADSC);
	while ( (ADCSRA & ~_BV(ADSC)) == 0 )
		continue;

	PRR |= _BV(PRUSI);	// we don't use USI
	PRR |= _BV(PRTIM0);	// we don't use TIMER0


	struct EEPROMData data;
	eeprom_read_block( &data, 0, sizeof(data) );
	uint16_t low_voltage_cutoff = data.low_voltage;
	uint8_t crc = calcCRC( low_voltage_cutoff );

	// if there is no stored low voltage value, or the switch is pressed, we go into calibrate mode
	if ( crc != data.crc ||
		 !(PINB & ~_BV(SWITCH_PIN)) )
	{
		// Average n samples
		uint16_t sum = 0;
		for ( int i = -10; i < SETUP_SAMPLES; i++ )
		{
			if ( i==0 ) sum = 0;

			PINB |= _BV(LED_PIN);

			ADCSRA |= _BV(ADSC);
			while ( (ADCSRA & ~_BV(ADSC)) == 0 )
				continue;
			sum += ADC;

			_delay_ms(100);
		}
		PORTB &= ~_BV(LED_PIN);
		low_voltage_cutoff = sum/SETUP_SAMPLES;
		data.crc = calcCRC( low_voltage_cutoff );
		data.low_voltage = low_voltage_cutoff;
		eeprom_write_block( &data, 0, sizeof(data) );
	}

	while (1)
	{
		ADCSRA |= _BV(ADSC);
		while ( (ADCSRA & ~_BV(ADSC)) == 0 )
			continue;

		uint16_t v = ADC;
		if ( v < low_voltage_cutoff )
		{
			PORTB |= _BV(LED_PIN);
			//PORTB |= _BV(BUZZER_PIN);
			TCCR1 |= _BV(CS10);					// Prescale /1
		}
		else
		{
			PORTB &= ~_BV(LED_PIN);
			//PORTB &= ~_BV(BUZZER_PIN);
			TCCR1 &= ~_BV(CS10);				// clk stopped
		}
	}
}

