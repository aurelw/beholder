/************************************************************************
 * This file is part of TerraControl.                                   *
 *                                                                      *
 * TerraControl is free software; you can redistribute it and/or modify *
 * it under the terms of the GNU General Public License as published    *
 * by the Free Software Foundation; either version 2 of the License, or *
 * (at your option) any later version.                                  *
 *                                                                      *
 * TerraControl is distributed in the hope that it will be useful,      *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of       *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        *
 * GNU General Public License for more details.                         *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with TerraControl; if not, write to the Free Software          *
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 *
 * USA                                                                  *
 * Written and (c) by mru                                               *
 * Contact <mru@sisyphus.teil.cc> for comment & bug reports             *
 ************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#include <stdlib.h>

#include "common.h"

#define UART_BAUD_RATE     38400

#define uchar unsigned char
#define uint unsigned int


#define US(x) ((x))

void moveto(int position /* 0..1 */) {
    const long min = 900;
    const long max = 2100;
    const int pos = min + (position * ( max - min ))/256;
       //   cli();
    OCR1B = US(pos);
         // sei();
}

int main(void)
{

  uint i=0;

  DDRD = 0b00000010;
  DDRB = 0b00010110; // PD4 als Ausgang

  // prescaler: cpuclock / 8
  // -> 2mhz

  TCCR1A = 0b00100011; // Ausgang OC1B bei 0 setzen und bei OCR1B loeschen
  TCCR1B = 0b00011010; // Waveform Generation Mode: Fast PWM it OCR1A
  // als Top, Timer mit CPU-CLK / 8

  //
  OCR1A = US(15000U); // PWM Periodenzeit in [msec]
  moveto(0);
 DDRB &= ~ ( _BV(PB4) | _BV(PB3) ); //PWM Periodenzeit in [msec]c= _BV(PB4) | _BV(PB3);



  uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,XTAL));
  sei();

  /* main loop section ---------------------------------- */

  int bs = 0, last_bs = 0;
  int stable_count = 0;
  for(;;) {

      int x = uart_getc();

      if ((x&0xff00) == 0) {
          moveto(x&0xff);
      }

      if ( !(PINB & _BV(PB4)) ) {
          if (bs == 2) stable_count++;
          else stable_count = 0;
          bs = 2;
      } 
      else if ( !(PINB & _BV(PB3) ) ) {
          if (bs == 1) stable_count++;
          else stable_count = 0;
          bs = 1;
      } 
      else {
          if (bs == 0) stable_count++;
          else stable_count = 0;
          bs = 0;
      }

      if ( last_bs != bs && stable_count > 100 ) {
        uart_putc('0' + bs); last_bs = bs;
        stable_count = 0;
      }

  }
}


/*  LocalWords:  eeprom
*/
