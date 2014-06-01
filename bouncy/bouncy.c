/* bouncy ball
 * 
 * for Teensy USB Development Board
 * http://www.pjrc.com/teensy/
 * Some parts Copyright (c) 2008 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
//#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usb_serial.h"
#include "sampling.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz	0x00
#define CPU_125kHz	0x07
#define HEX(n) (((n) < 10) ? ((n) + '0') : ((n) + 'A' - 10))

// Teensy 2.0: LED is active high
#if defined(__AVR_ATmega32U4__) || defined(__AVR_AT90USB1286__)
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define LED_TOGGLE	(PORTD ^= (1<<6))
#endif

#define LED_CONFIG	(DDRD |= (1<<6))


void blink_n_times(int v) {
  LED_OFF;
  v *= 2;
  while (v-- > 0) {
    _delay_ms(100);
    LED_TOGGLE;
  }
  _delay_ms(200);
}
  
void init_slug_driver(void) {
  // Set up the H-bridge - a Vishay Si9986
  // It's connected to PC6 and PC7
  PORTC = 0;
  DDRC = 0xC0;
}

void drive_slug_left(void) {
  PORTC |=  0x40;
  PORTC &= ~0x80;
}

void drive_slug_right(void) {
  PORTC |=  0x80;
  PORTC &= ~0x40;
}

void coast_slug(void) {
  PORTC |= 0xc0;
}

void brake_slug(void) {
  PORTC &= ~0xc0;
}

void init_slug_sensors(void) {
  DDRB &= ~((1<<7) | 0xf);
  PORTB &= ~0x0f;
}

int read_slug_sensor(int n) {
  int v;
  n &= 3;
  PORTB &= ~(1<<n);
  DDRB |= 1<<n;
  _delay_us(200);
  v = PINB & (1<<7);
  DDRB &= ~(1<<n);
#if 0
          char buf[6];
	  buf[0] = '0' + n;
	  buf[1] = ':';
	  buf[2] = HEX((v >> 4) & 0xf);
	  buf[3] = HEX(v & 0xf);
	  buf[4] = '\r';
	  buf[5] = '\n';
	  usb_serial_write((unsigned char *)buf, 6);
#endif
  return v ? 0 : 1;
}

void init_remote(void) {
  DDRD &= ~(1<<0);
  PORTD |= 1; // pullup
}

int remote_signal(void) {
  return PIND & (1<<0) ? 0 : 1;
}

int main(void)
{
  CPU_PRESCALE(CPU_125kHz);
  _delay_ms(1);		// allow slow power supply startup
  CPU_PRESCALE(CPU_16MHz); // set for 16 MHz clock
  //	CPU_PRESCALE(0);

  LED_CONFIG;
  LED_OFF;

  // initialize the USB, and then wait for the host
  // to set configuration.  If the Teensy is powered
  // without a PC connected to the USB port, this 
  // will wait forever.
  usb_init();
  //while (!usb_configured()) /* wait */ ;
  //_delay_ms(1000);


  // Initialize
  init_slug_driver();
  init_slug_sensors();
  init_remote();
	
  while (1) {
    if (remote_signal()) {
      LED_ON;
    } else {
      LED_OFF;
    }
  }    


  // A little test
  for (int i=0; ; i++) {
    if (i&1) {
      drive_slug_left();
    } else {
      drive_slug_right();
    }
    _delay_ms(200);
    coast_slug();
    for (int n=0; n<4; n++) {
      if (read_slug_sensor(n)) {
	blink_n_times(n+1);
      }
    }
  }
}


