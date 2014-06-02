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
#include <stdlib.h>
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
  
void delay_us(int delay) {
  if (delay < 1) return;
  if (delay & (1<<0)) _delay_us(1<<0);
  if (delay & (1<<1)) _delay_us(1<<1);
  if (delay & (1<<2)) _delay_us(1<<2);
  if (delay & (1<<3)) _delay_us(1<<3);
  if (delay & (1<<4)) _delay_us(1<<4);
  if (delay & (1<<5)) _delay_us(1<<5);
  if (delay & (1<<6)) _delay_us(1<<6);
  if (delay & (1<<7)) _delay_us(1<<7);
  for (int n=delay>>8; n; n--) _delay_us(1<<8);
}


void init_slug_driver(void) {
  // Set up the H-bridge - a Vishay Si9986
  // It's connected to PC6 and PC7
  PORTC = 0;
  DDRC = 0xC0;
}

void drive_coil_in(void) {
  PORTC |=  0x40;
  PORTC &= ~0x80;
}

void drive_coil_out(void) {
  PORTC |=  0x80;
  PORTC &= ~0x40;
}

void OC_coil(void) {
  PORTC |= 0xc0;
}

void CC_coil(void) {
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

int drive_slug_left(int t) {
  drive_coil_out();
  while ((!read_slug_sensor(0) && !read_slug_sensor(3)) && 0 < t) {
    _delay_ms(1);
    t = t - 1;
  }
  if (read_slug_sensor(0)) {
    drive_coil_in();
    while (!read_slug_sensor(1) && 0 < t) {
      _delay_ms(1);
      t = t - 1;
    }
    while (read_slug_sensor(1) && 0 < t) {
      _delay_ms(1);
      t = t - 1;
    }
    OC_coil();
    while (!read_slug_sensor(2) && 0 < t) {
      _delay_ms(1);
      t = t - 1;
    }
    drive_coil_out();
    while (!read_slug_sensor(3) && 0 < t) {
      _delay_ms(1);
      t = t - 1;
    }
    CC_coil();
    return 1;
      }else{
    while (!read_slug_sensor(3) && 0 < t) {
      _delay_ms(1); 
      t = t - 1;
    }
    CC_coil();
    return 0;
      }
}

int drive_slug_right(int t) {
  drive_coil_out();
  while ((!read_slug_sensor(0) && !read_slug_sensor(3)) && 0 < t) {
    _delay_ms(1);
    t = t - 1;
  }
  if (read_slug_sensor(3)) {
    drive_coil_in();
    while (!read_slug_sensor(2) && 0 < t) {
      _delay_ms(1);
      t = t - 1;
    }
    while (read_slug_sensor(2) && 0 < t) {
      _delay_ms(1);
      t = t - 1;
    }
    OC_coil();
    while (!read_slug_sensor(1) && 0 < t) {
      _delay_ms(1);
      t = t - 1;
    }
    drive_coil_out();
    while (!read_slug_sensor(0) && 0 < t) {
      _delay_ms(1);
      t = t - 1;
    }
    CC_coil();
    return 1;
      }else{
    while (!read_slug_sensor(0) && 0 < t) {
      _delay_ms(1);
      t = t - 1;
    }
    CC_coil();
    return 0;
      }
}

void oscillate(float F,int N) {
  int period = (500000 / F);
  while (0 < N) {
    drive_coil_out();
    delay_us(period);
    drive_coil_in();
    delay_us(period);
    N = N - 1;
  }
  CC_coil();
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
  
  int win = 1;
  int level = 15;

  while (1) {
    if (remote_signal()) {
      LED_ON;
      win = win - 1;
      if (win > 0) {
	if (!drive_slug_left(500)){
	  drive_slug_right(500);
	}
      }else{
	while (win < level) {
	  oscillate(440,110);
	  _delay_ms(750);
	  win = win + (level / 3);
	}
	level = level * 1.5;
      }
    } else {
      LED_OFF;
      if (win < level) {
	win = win + (level / 10);
      }
      // oscillate(440,10);
      if (((rand()%31) == 5) && 0) {
	oscillate((rand()%18 + 4),(rand()%5 + 2));
      }
    }
  }    


  // A little test
  for (int i=0; ; i++) {
    if (i&1) {
      drive_slug_left(500);
    } else {
      drive_slug_right(500);
    }
    _delay_ms(200);
    OC_coil();
    for (int n=0; n<4; n++) {
      if (read_slug_sensor(n)) {
	blink_n_times(n+1);
      }
    }
  }
}


