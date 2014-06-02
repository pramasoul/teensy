
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "sampling.h"

#define BUFSIZE 50

// Teensy 2.0: LED is active high
#if defined(__AVR_ATmega32U4__) || defined(__AVR_AT90USB1286__)
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define LED_TOGGLE	(PORTD ^= (1<<6))
#endif

#define LED_CONFIG	(DDRD |= (1<<6))


static volatile uint8_t head, tail;
static volatile int16_t buffer[BUFSIZE];

void adc_start(uint8_t mux, uint8_t aref)
{
	ADCSRA = (1<<ADEN) | ADC_PRESCALER;	// enable the ADC, interrupt disabled
	ADCSRB = (1<<ADHSM) | (mux & 0x20);
	ADMUX = aref | (mux & 0x1F);		// configure mux and ref
	head = 0;				// clear the buffer
	tail = 0;				// and then begin auto trigger mode
	ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | ADC_PRESCALER;
	sei();
}

uint8_t adc_available(void)
{
	uint8_t h, t;

	h = head;
	t = tail;
	if (h >= t) return h = t;
	return BUFSIZE + h - t;
}

int16_t adc_read(void)
{
	uint8_t h, t;
	int16_t val;

	do {
		h = head;
		t = tail;		// wait for data in buffer
	} while (h == t);
	if (++t >= BUFSIZE) t = 0;
	val = buffer[t];		// remove 1 sample from buffer
	tail = t;
	return val;
}


static uint16_t prescaler;
int32_t in_phase, quadrature;
uint8_t signal_present;
static int32_t in_phase_acc, quadrature_acc;

int32_t damp_toward_zero(int32_t v) {
  int32_t rv;
  //rv = ((v<<8) - v + ((v<0 ? -1 : 1) << 7)) >> 8;
  rv = ((v<<8) - v) >> 8;
  return rv;
}

ISR(ADC_vect)
{
	uint8_t h;
	int16_t val;
	val = ADC;			// grab new reading from ADC
	prescaler = (prescaler + 1) & 0x3ff;
	if (prescaler == 0) {
	  in_phase = in_phase_acc;
	  quadrature = quadrature_acc;
	  in_phase_acc = quadrature_acc = 0;
	  signal_present = abs(in_phase) + abs(quadrature) > 200;
	  if (signal_present) LED_ON; else LED_OFF;
	}
	if (prescaler & 1) {
	  PORTD ^= (1<<5); // Toggle the output if we're the transmitter
	}
	switch (prescaler & 3) {
	case 0:
	  in_phase_acc += val;
	  break;
	case 1:
	  quadrature_acc += val;
	  break;
	case 2:
	  in_phase_acc -= val;
	  break;
	case 3:
	  quadrature_acc -= val;
	  break;
	}
	h = head + 1;
	if (h >= BUFSIZE) h = 0;
	if (h != tail) {		// if the buffer isn't full
		buffer[h] = val;	// put new data into buffer
		head = h;
	}
}

