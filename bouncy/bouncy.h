#ifndef _bouncy_h_included__
#define _bouncy_h_included__

// Teensy 2.0: LED is active high
#if defined(__AVR_ATmega32U4__) || defined(__AVR_AT90USB1286__)
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define LED_TOGGLE	(PORTD ^= (1<<6))
#endif

#define LED_CONFIG	(DDRD |= (1<<6))

#endif
