/*
soft_serial.c (formerly SoftwareSerial.cpp (formerly NewSoftSerial.cpp)) - 
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

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 REGISTER_BIT(PORTB,3)
#define _DEBUG_PIN2 REGISTER_BIT(PORTB,5)

// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "soft_serial.h"

#include "config.h"
#include "io_cfg.h"
#include "settings.h"
#include "gyros.h"

#ifdef SERIAL_PORT

static uint16_t ss_rx_delay_centering;
static uint16_t ss_rx_delay_intrabit;
static uint16_t ss_rx_delay_stopbit;
static uint16_t ss_tx_delay;

static uint8_t ss_buffer_overflow;
static uint8_t ss_inverse_logic;
static uint8_t ss_initialized = FALSE;

static char ss_receive_buffer[SS_MAX_RX_BUFF]; 
static volatile uint8_t ss_receive_buffer_tail;
static volatile uint8_t ss_receive_buffer_head;

//
// Lookup table
//
typedef struct _DELAY_TABLE
{
  long baud;
  int16_t rx_delay_centering;
  int16_t rx_delay_intrabit;
  int16_t rx_delay_stopbit;
  int16_t tx_delay;
} DELAY_TABLE;

#if F_CPU == 16000000

static const DELAY_TABLE PROGMEM table[] = 
{
  //  baud    rxcenter   rxintra    rxstop    tx
  { 115200,   1,         17,        17,       12,    },
  { 57600,    10,        37,        37,       33,    },
  { 38400,    25,        57,        57,       54,    },
  { 31250,    31,        70,        70,       68,    },
  { 28800,    34,        77,        77,       74,    },
  { 19200,    54,        117,       117,      114,   },
  { 14400,    74,        156,       156,      153,   },
  { 9600,     114,       236,       236,      233,   },
  { 4800,     233,       474,       474,      471,   },
  { 2400,     471,       950,       950,      947,   },
  { 1200,     947,       1902,      1902,     1899,  },
  { 300,      3804,      7617,      7617,     7614,  },
};

const int XMIT_START_ADJUSTMENT = 5;

#elif F_CPU == 8000000

static const DELAY_TABLE table[] PROGMEM = 
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   1,          5,         5,      6,      },
  { 57600,    1,          15,        15,     15,     },
  { 38400,    2,          25,        26,     23,     },
  { 31250,    7,          32,        33,     29,     },
  { 28800,    11,         35,        35,     32,     },
  { 19200,    20,         55,        55,     52,     },
  { 14400,    30,         75,        75,     72,     },
  { 9600,     50,         114,       114,    112,    },
  { 4800,     110,        233,       233,    230,    },
  { 2400,     229,        472,       472,    469,    },
  { 1200,     467,        948,       948,    945,    },
  { 300,      1895,       3805,      3805,   3802,   },
};

const int XMIT_START_ADJUSTMENT = 4;

#elif F_CPU == 20000000

// 20MHz support courtesy of the good people at macegr.com.
// Thanks, Garrett!

static const DELAY_TABLE PROGMEM table[] =
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   3,          21,        21,     18,     },
  { 57600,    20,         43,        43,     41,     },
  { 38400,    37,         73,        73,     70,     },
  { 31250,    45,         89,        89,     88,     },
  { 28800,    46,         98,        98,     95,     },
  { 19200,    71,         148,       148,    145,    },
  { 14400,    96,         197,       197,    194,    },
  { 9600,     146,        297,       297,    294,    },
  { 4800,     296,        595,       595,    592,    },
  { 2400,     592,        1189,      1189,   1186,   },
  { 1200,     1187,       2379,      2379,   2376,   },
  { 300,      4759,       9523,      9523,   9520,   },
};

const int XMIT_START_ADJUSTMENT = 6;

#else

#error This version of SoftwareSerial supports only 20, 16 and 8MHz processors

#endif

//
// Statics
//
static char ss_receive_buffer[SS_MAX_RX_BUFF]; 
static volatile uint8_t ss_receive_buffer_tail = 0;
static volatile uint8_t ss_receive_buffer_head = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
inline void DebugPulse(uint8_t pin, uint8_t count)
{
#if _DEBUG
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
#endif
}

/* static */ 
inline void ss_tunedDelay(uint16_t delay) { 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+r" (delay), "+a" (tmp)
    : "0" (delay)
    );
}

uint8_t ss_listen()
{
  uint8_t oldSREG = SREG;
  ss_buffer_overflow = false;
  cli();
  ss_receive_buffer_head = ss_receive_buffer_tail = 0;
  SREG = oldSREG;

  ss_initialized = TRUE;

  return true;
}

uint8_t ss_isListening(void) 
{
    return ss_initialized;
}

//
// The receive routine called by the interrupt handler
//
void ss_recv()
{

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Preserve the registers that the compiler misses
// (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t"
    ::);
#endif  

  uint8_t d = 0;
  uint8_t val = 0;
  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  #if defined(SERIAL_RX_M3)
  val = REGISTER_BIT( PINB, 0 );
  #elif defined(SERIAL_RX_M4)
  val = REGISTER_BIT( PIND, 7 );
  #elif defined(SERIAL_RX_M5)
  val = REGISTER_BIT( PIND, 6 );
  #elif defined(SERIAL_RX_M6)
  val = REGISTER_BIT( PIND, 5 );
  #endif
  if (ss_inverse_logic ? val : !val )
  {
    // Wait approximately 1/2 of a bit width to "center" the sample
    ss_tunedDelay(ss_rx_delay_centering);
    DebugPulse(_DEBUG_PIN2, 1);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1)
    {
      ss_tunedDelay(ss_rx_delay_intrabit);
      DebugPulse(_DEBUG_PIN2, 1);
      uint8_t noti = ~i;
      #if defined(SERIAL_RX_M3)
      val = REGISTER_BIT( PINB, 0 );
      #elif defined(SERIAL_RX_M4)
      val = REGISTER_BIT( PIND, 7 );
      #elif defined(SERIAL_RX_M5)
      val = REGISTER_BIT( PIND, 6 );
      #elif defined(SERIAL_RX_M6)
      val = REGISTER_BIT( PIND, 5 );
      #endif
      
      if ( val )
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    
    }

    // skip the stop bit
    ss_tunedDelay(ss_rx_delay_stopbit);
    DebugPulse(_DEBUG_PIN2, 1);
    
    if (ss_inverse_logic)
      d = ~d;

    // if buffer full, set the overflow flag and return
    if ((ss_receive_buffer_tail + 1) % SS_MAX_RX_BUFF != ss_receive_buffer_head) 
    {
      // save new data in buffer: tail points to where byte goes
      ss_receive_buffer[ss_receive_buffer_tail] = d; // save new byte
      ss_receive_buffer_tail = (ss_receive_buffer_tail + 1) % SS_MAX_RX_BUFF;
    } 
    else 
    {
#if _DEBUG // for scope: pulse pin as overflow indictator
      DebugPulse(_DEBUG_PIN1, 1);
      _DEBUG_PIN1 ^= 1;
#endif
      ss_buffer_overflow = true;
    }

    #if _DEBUG
    ss_write( (char)d );
    ss_write( ':' );
    ss_write_hex( (int)d );
    ss_write( '\n' );
    ss_write( '\r' );
    #endif
  }
  
#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);
#endif
}

void ss_tx_pin_write(uint8_t pin_state)
{
    SER_TX = pin_state;
}

uint8_t ss_rx_pin_read()
{
  return SER_RX;
}

void soft_serial( uint8_t inverse_logic ) 
{
  ss_rx_delay_centering = 0;
  ss_rx_delay_intrabit = 0;
  ss_rx_delay_stopbit = 0;
  ss_tx_delay = 0;
  ss_buffer_overflow = false;
  ss_inverse_logic = inverse_logic;

  #if defined(SERIAL_RX_M3)
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT0);      // PB0
  #elif defined(SERIAL_RX_M4)
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT23);      // PD7
  #elif defined(SERIAL_RX_M5)
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT22);      // PD6
  #elif defined(SERIAL_RX_M6)
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT21);      // PD5
  #endif

  ss_setTX();
  ss_setRX();
}

void ss_setTX()
{
  SER_TX_DIR = OUTPUT;
  SER_TX = HIGH;
}

void ss_setRX()
{
  SER_RX_DIR = INPUT;
  if (!ss_inverse_logic)
    SER_RX = HIGH;  // pullup for normal logic!
}

void ss_begin(long speed)
{
  ss_rx_delay_centering = ss_rx_delay_intrabit = ss_rx_delay_stopbit = ss_tx_delay = 0;

  for (unsigned i=0; i<sizeof(table)/sizeof(table[0]); ++i)
  {
    long baud = pgm_read_dword(&table[i].baud);
    if (baud == speed)
    {
      ss_rx_delay_centering = pgm_read_word(&table[i].rx_delay_centering);
      ss_rx_delay_intrabit = pgm_read_word(&table[i].rx_delay_intrabit);
      ss_rx_delay_stopbit = pgm_read_word(&table[i].rx_delay_stopbit);
      ss_tx_delay = pgm_read_word(&table[i].tx_delay);
      break;
    }
  }

  // Set up RX interrupts, but only if we have a valid RX baud rate
  if (ss_rx_delay_stopbit)
  {
#ifdef SERIAL_RX_M4
      PCMSK2 = _BV( PCINT23 );
#elif defined(  SERIAL_RX_M5 )
      PCMSK2 |= _BV( PCINT22 );
#elif defined(  SERIAL_RX_M6 )
      PCMSK2 |= _BV( PCINT21 );
#elif defined(  SERIAL_RX_M3 )
      PCMSK0 |= _BV( PCINT0 );
#endif

    ss_tunedDelay(ss_tx_delay); // if we were low this establishes the end
  }

  REGISTER_BIT(DDRB,3) = OUTPUT;
  REGISTER_BIT(DDRB,5) = OUTPUT;

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif

  ss_listen();
}

void ss_end()
{
#ifdef SERIAL_RX_M4
      PCMSK2 &= ~_BV( PCINT23 );
#elif defined(  SERIAL_RX_M5 )
      PCMSK2 &= ~_BV( PCINT22 );
#elif defined(  SERIAL_RX_M6 )
      PCMSK2 &= ~_BV( PCINT21 );
#elif defined(  SERIAL_RX_M3 )
      PCMSK0 &= ~_BV( PCINT0 );
#endif
}

// Read data from buffer
int ss_read()
{
  if (!ss_isListening())
    return -1;

  // Empty buffer?
  if (ss_receive_buffer_head == ss_receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = ss_receive_buffer[ss_receive_buffer_head]; // grab next byte
  ss_receive_buffer_head = (ss_receive_buffer_head + 1) % SS_MAX_RX_BUFF;
  return d;
}

int ss_available()
{
  if (!ss_isListening())
    return 0;

  return (ss_receive_buffer_tail + SS_MAX_RX_BUFF - ss_receive_buffer_head) % SS_MAX_RX_BUFF;
}

size_t ss_write_str( char *buf )
{
    while( *buf != 0 ) {
	ss_write( *buf++ );
	ss_tunedDelay(ss_tx_delay<<4); 
    }
}

size_t ss_write_num( int n )
{
    char buf[11];
    int i;

    buf[10] = 0;

    if( n != 0 ) {
	for( i=9; i>0 && n>0; i-- )
	    {
		buf[i] = n%10 + '0';
		n = n/10;
	    }
    } else {
	buf[9] = '0';
	i=8;
    }

    /* any good C book will tell you not to use the */
    /* value of a for loop variable after a loop. That's */
    /* really good advice however it's ignored here */
    ss_write_str( &buf[++i] );

    return 9 - i;
}

size_t ss_write_hex( int n )
{
  char buf[9];
  int i, rem;

  buf[8] = 0;

  if( n != 0 ) {
	  for( i=7; i>0 && n>0; i-- )
	  {
		  rem = n%16;
		  if( rem < 10 )
		    buf[i] = rem + '0';
		  else
		    buf[i] = rem - 10 + 'A';

		  n = n/16;
	  }
  } else {
	  buf[7] = '0';
	  i=6;
  }

    /* any good C book will tell you not to use the */
    /* value of a for loop variable after a loop. That's */
    /* really good advice however it's ignored here */
    ss_write_str( &buf[++i] );

    return 9 - i;
}

size_t ss_write(uint8_t b)
{
  if (ss_tx_delay == 0) {
    return 0;
  }

  uint8_t oldSREG = SREG;
  cli();  // turn off interrupts for a clean txmit

  // Write the start bit
  ss_tx_pin_write(ss_inverse_logic ? HIGH : LOW);
  ss_tunedDelay(ss_tx_delay + XMIT_START_ADJUSTMENT);

  // Write each of the 8 bits
  if (ss_inverse_logic)
  {
    for (unsigned char mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        ss_tx_pin_write(LOW); // send 1
      else
        ss_tx_pin_write(HIGH); // send 0
    
      ss_tunedDelay(ss_tx_delay);
    }

    ss_tx_pin_write(LOW); // restore pin to natural state
  }
  else
  {
    for (unsigned char mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        ss_tx_pin_write(HIGH); // send 1
      else
        ss_tx_pin_write(LOW); // send 0
      ss_tunedDelay(ss_tx_delay);
    }

    ss_tx_pin_write(HIGH); // restore pin to natural state
  }

  SREG = oldSREG; // turn interrupts back on
  ss_tunedDelay(ss_tx_delay);
  
  return 1;
}

void ss_flush()
{
  if (!ss_isListening())
    return;

  uint8_t oldSREG = SREG;
  cli();
  ss_receive_buffer_head = ss_receive_buffer_tail = 0;
  SREG = oldSREG;
}

int ss_peek()
{
  if (!ss_isListening())
    return -1;

  // Empty buffer?
  if (ss_receive_buffer_head == ss_receive_buffer_tail)
    return -1;

  // Read from "head"
  return ss_receive_buffer[ss_receive_buffer_head];
}

void setLinvorRate( int32_t rate )
{
  char buffer[16], rate_str[16];
  int i = 0;
  int c = 0;

  ss_write_str( "AT\r\n" );
  _delay_ms(1000);
  do {
    c = ss_read();
    if( c != -1 && i < 16 ) 
      buffer[i++] = c;
  } while( c != -1 );

  switch( rate ) {
    case 1200:
      strcpy( rate_str, "AT+BAUD1\r\n" );
      break;
    case 2400:
      strcpy( rate_str, "AT+BAUD2\r\n" );
      break;
    case 4800:
      strcpy( rate_str, "AT+BAUD3\r\n" );
      break;
    case 9600:
      strcpy( rate_str, "AT+BAUD4\r\n" );
      break;
    case 19200:
      strcpy( rate_str, "AT+BAUD5\r\n" );
      break;
    case  38400:
      strcpy( rate_str, "AT+BAUD6\r\n" );
      break;
    case 57600:
      strcpy( rate_str, "AT+BAUD7\r\n" );
      break;
    case 115200L: 
      strcpy( rate_str, "AT+BAUD8\r\n" );
      break;
#ifdef UART_SERIAL
    case 230400L: 
      strcpy( rate_str, "AT+BAUD9\r\n" );
      break;
    case 460800L:
      strcpy( rate_str, "AT+BAUDA\r\n" );
      break;
    case 921600L: 
      strcpy( rate_str, "AT+BAUDB\r\n" );
      break;
    case 1382400L:
      strcpy( rate_str, "AT+BAUDC\r\n" );
      break;
#endif
  }

  if( !strncmp( buffer, "OK", 2 ))
  {
    i = 0;
    ss_write_str( rate_str );
    _delay_ms(1000);
    do {
      c = ss_read();
      if( c != -1 && i < 16 ) 
        buffer[i++] = c;
    } while( c != -1 );
  }
  else
  {
    ss_write_str( "Baud rate setting failed" );
  }

  if( !strncmp( buffer, "OK", 2 ))
    soft_serial( 115200L );
}

void menu( void )
{
  int16_t c;

  c = ss_read();
  if( c != -1 ) {
    switch( c ) {
    case 'g':
      ReadGainPots();
      ss_write_str( "\r\nPitch gain   = " );
      ss_write_num( GainInADC[PITCH] );
      ss_write_str( "\r\nRoll gain    = " );
      ss_write_num( GainInADC[ROLL] );
      ss_write_str( "\r\nYaw gain     = " );
      ss_write_num( GainInADC[YAW] );
      ss_write( '\r\n' );
      break;
    case 'G':
      ReadGyros();
      ss_write_str( "\r\nPitch gyro   = " );
      ss_write_num( gyroADC[PITCH] );
      ss_write_str( "\r\nRoll gyro    = " );
      ss_write_num( gyroADC[ROLL] );
      ss_write_str( "\r\nYaw gyro     = " );
      ss_write_num( gyroADC[YAW] );
      ss_write( '\r\n' );
      break;
#ifdef SAVE_CENTER
    case 'S':
      // this doesn't return
      receiverStickCenter();
      break;
    case 's':
      ss_write_str( "\r\nRoll Center       = " );
      ss_write_num( Config.CenterRollValue );
      ss_write_str( "\r\nPitch Center      = " );
      ss_write_num( Config.CenterPitchValue );
      ss_write_str( "\r\nCollector Center  = " );
      ss_write_num( Config.CenterCollValue );
      ss_write_str( "\r\nYaw Center         = " );
      ss_write_num( Config.CenterYawValue );
      break;
#endif
    }
  }
}
#endif
