/* Hacking around with laser comm
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2008 PJRC.COM, LLC
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

#include <stdlib.h>

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
  _delay_ms(100);
}
  
int putchar_to_usb(char c) {
  return usb_serial_write((unsigned char *) &c, 1);
}

int print_int_to_usb(int v) {
  char *p, buf[12];
  int rv = 0;
  if (v<0) {
    putchar_to_usb('-');
    v *= -1;
  }
  p = buf;
  while (v >=10) {
    *p++ = '0' + v%10;
    v /= 10;
  }
  *p = '0' + v;
  while (p >= buf) {
    rv = putchar_to_usb(*p--);
    if (rv) break;
  }
  return rv;
}

int print_int32_to_usb(int32_t v) {
  char *p, buf[12];
  int rv = 0;
  if (v<0) {
    putchar_to_usb('-');
    v *= -1;
  }
  p = buf;
  while (v >=10) {
    *p++ = '0' + v%10;
    v /= 10;
  }
  *p = '0' + v;
  while (p >= buf) {
    rv = putchar_to_usb(*p--);
    if (rv) break;
  }
  return rv;
}

int main(void)
{
	uint16_t val;
	char buf[16];

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
	//	while (!usb_configured()) /* wait */ ;
	_delay_ms(1000);

	DDRD |= (1<<5);

	adc_start(ADC_MUX_PIN_D4, ADC_REF_POWER);
	//adc_start(ADC_MUX_PIN_D4, ADC_REF_INTERNAL);

	while (0) {
	  print_int_to_usb(rand());
	  putchar_to_usb('\r');
	  putchar_to_usb('\n');
	}

	while (0) {
	  // read the next ADC sample, and send it as ascii hex
	  val = adc_read();
	  //_delay_ms(500);//DEBUG
	  PORTD ^= (1<<5);
	  LED_TOGGLE;
	  buf[0] = HEX((val >> 8) & 15);
	  buf[1] = HEX((val >> 4) & 15);
	  buf[2] = HEX(val & 15);
	  buf[3] = ' ';
	  int v = usb_serial_write((unsigned char *)buf, 4);
	  if (v) {
	    blink_n_times(0-v);
	  }
	}

	if (0) {
	  extern int32_t damp_toward_zero(int32_t v);
	  int32_t t = -123456;
	  for (int i=0; i<2048; i++) {
	    putchar_to_usb('\r');
	    putchar_to_usb('\n');
	    print_int32_to_usb(t);
	    _delay_ms(30);
	    t = damp_toward_zero(t);
	  }
	} 

	while (1) {
	  extern int32_t in_phase, quadrature;
	  int v;
	  LED_TOGGLE;
	  _delay_ms(100);
	  putchar_to_usb('\r');
	  putchar_to_usb('\n');
	  if ((v = print_int_to_usb(in_phase)) != 0
	      || (v = putchar_to_usb(' ')) != 0
	      || (v = print_int_to_usb(quadrature)) != 0) {
	    blink_n_times(0-v);
	  }
	}
}


