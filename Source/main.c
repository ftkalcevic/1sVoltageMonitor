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
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/cpufunc.h>

#define LED_PIN		PB0
#define BUZZER_PIN	PB4
#define ADC_PIN		PB3
#define SWITCH_PIN	PB1

#define UNUSED_PINS	(_BV(PB2) | _BV(PB5))

#define SETUP_SAMPLES 16


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

ISR(WDT_vect)
{
	wdt_reset();
}

int main(void)
{
	CLKPR = _BV(CLKPCE);
	CLKPR = _BV(CLKPS2) | _BV(CLKPS1);

	// Outputs
	DDRB |= _BV(LED_PIN) | _BV(BUZZER_PIN);
	// Pullup on switch input
	PORTB |= _BV(SWITCH_PIN) | UNUSED_PINS;

	// Set up ADC
	ADMUX = _BV(REFS2) | _BV(REFS1) |	// Internal 2.56v reference
			0 |							// Right adjust result
			_BV(MUX1) | _BV(MUX0);		// ADC3(PB3)
	ADCSRA = _BV(ADEN) |				// ADC Enable
			0 |							// manual trigger
			0 |							// No interrupts
			0;							// Prescale x2
	ADCSRB = 0;							
	DIDR0 = _BV(ADC3D);
	_delay_ms(1);

	// Setup tranducer timer1 -4.4kHz
	TCCR1 = _BV(CTC1) |					// Clear counter on OCR1C match
			0;							// clock stopped
	GTCCR = _BV(PWM1B) |				// PWM mode
			_BV(COM1B0);				// toggle oc1b
	OCR1B = 14;
	OCR1C = 28;


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

	sei();
	set_sleep_mode(SLEEP_MODE_IDLE);

	uint8_t beeper = 0;
	while (1)
	{
		// Watchdog reset (wake from sleep)
		ADCSRA |= _BV(ADSC);
		while ( (ADCSRA & ~_BV(ADSC)) == 0 )
			continue;
		ADCSRA |= _BV(ADSC);
		while ( (ADCSRA & ~_BV(ADSC)) == 0 )
			continue;

		uint16_t v = ADC;
		if ( v < low_voltage_cutoff )
		{
			PORTB |= _BV(LED_PIN);
			//PORTB |= _BV(BUZZER_PIN);
			if ( beeper & 1 )
				TCCR1 &= ~_BV(CS10);				// clk stopped
			else
				TCCR1 |= _BV(CS10);					// Prescale /1
			beeper++;
		}
		else
		{
			PORTB &= ~_BV(LED_PIN);
			//PORTB &= ~_BV(BUZZER_PIN);
			TCCR1 &= ~_BV(CS10);				// clk stopped
			beeper = 0;
		}
		
		WDTCR = _BV(WDE) |					// Watch dog enable
				_BV(WDIE) |					// watch dog interrupt
				_BV(WDP1) | _BV(WDP0);		// 0.125ms
		sleep_mode();	// we'll be woken by the watchdog.
	}
}

