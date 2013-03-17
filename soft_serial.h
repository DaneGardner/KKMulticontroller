/*
soft_serial. (formerly SoftwareSerial.h (formerly NewSoftSerial.h)) - 
software serial library for AVR microcontrollers
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer,
   porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

#ifndef _soft_serial_h_
#define _soft_serial_h_

#include <inttypes.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define SS_MAX_RX_BUFF 64 // RX buffer size

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

uint8_t ss_listen(void);
void ss_recv(void);
uint8_t ss_rx_pin_read(void);
void ss_setTX(void);
void ss_setRX(void);
void ss_end(void);
int ss_read(void);
uint8_t ss_isListening(void);
void ss_tunedDelay(uint16_t delay);
int ss_available(void);
void soft_serial(uint8_t inverse_logic);
void ss_begin(long speed);
size_t ss_write_str(char *buf);
size_t ss_write(uint8_t b);
size_t ss_write_num( int n );
size_t ss_write_hex( int n );
int ss_peek(void);
void ss_flush(void);
void setLinvorRate( int32_t rate );
void menus( void );

#define HIGH 1
#define LOW  0

#endif
