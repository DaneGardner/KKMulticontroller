#include "receiver.h"
#include "settings.h"

/*** BEGIN VARIABLES ***/
int16_t RxInRoll;
int16_t RxInPitch;
int16_t RxInCollective;
int16_t RxInYaw;

/*
 * Careful! Making these volatile makes it spew r24 crap in the _middle_ of
 * asm volatile statements. Always check assembler output of interrupt
 * routines.
 */
uint16_t RxChannel1;
uint16_t RxChannel2;
uint16_t RxChannel3;
uint16_t RxChannel4;

uint16_t CenterRollValue;
uint16_t CenterPitchValue;
uint16_t CenterCollValue;
uint16_t CenterYawValue;

#ifdef TWIN_COPTER
int16_t RxInOrgPitch;
#endif
/*** END VARIABLES ***/

/*** BEGIN RECEIVER INTERRUPTS ***/
/*
 * Rx interrupts with inline assembler that makes them much faster.
 * Please verify that GCC does not inject anything crazy in here,
 * such as completely unused movs that clobber other registers.
 */

ISR(PCINT2_vect, ISR_NAKED)
{
  if(RX_ROLL) {        // rising
    asm volatile("lds %A0, %1" : "=r" (RxChannel1Start) : "i" (&TCNT1L));
    asm volatile("lds %B0, %1" : "=r" (RxChannel1Start) : "i" (&TCNT1H));
    asm volatile("reti");
  } else {        // falling
    asm volatile(
      "lds %A0, %3\n"
      "lds %B0, %4\n"
      "in %1, __SREG__\n"
      "sub %A0, %A2\n"
      "sbc %B0, %B2\n"
      "out __SREG__, %1\n"
        : "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel1Start)
        : "i" (&TCNT1L), "i" (&TCNT1H));
    RxChannel1 = i_tmp;
  }
  asm volatile ("reti");
}

ISR(INT0_vect, ISR_NAKED)
{
  if(RX_PITCH) {        // rising
    asm volatile("lds %A0, %1" : "=r" (RxChannel2Start) : "i" (&TCNT1L));
    asm volatile("lds %B0, %1" : "=r" (RxChannel2Start) : "i" (&TCNT1H));
    asm volatile("reti");
  } else {        // falling
    asm volatile(
      "lds %A0, %3\n"
      "lds %B0, %4\n"
      "in %1, __SREG__\n"
      "sub %A0, %A2\n"
      "sbc %B0, %B2\n"
      "out __SREG__, %1\n"
        : "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel2Start)
        : "i" (&TCNT1L), "i" (&TCNT1H));
    RxChannel2 = i_tmp;
  }
  asm volatile ("reti");
}

ISR(INT1_vect, ISR_NAKED)
{
  if(RX_COLL) {        // rising
    asm volatile("lds %A0, %1" : "=r" (RxChannel3Start) : "i" (&TCNT1L));
    asm volatile("lds %B0, %1" : "=r" (RxChannel3Start) : "i" (&TCNT1H));
    asm volatile("reti");
  } else {        // falling
    asm volatile(
      "lds %A0, %3\n"
      "lds %B0, %4\n"
      "in %1, __SREG__\n"
      "sub %A0, %A2\n"
      "sbc %B0, %B2\n"
      "out __SREG__, %1\n"
        : "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel3Start)
        : "i" (&TCNT1L), "i" (&TCNT1H));
    RxChannel3 = i_tmp;
  }
  asm volatile ("reti");
}

ISR(PCINT0_vect, ISR_NAKED)
{
  if(RX_YAW) {        // rising
    asm volatile("lds %A0, %1" : "=r" (RxChannel4Start) : "i" (&TCNT1L));
    asm volatile("lds %B0, %1" : "=r" (RxChannel4Start) : "i" (&TCNT1H));
    asm volatile("reti");
  } else {        // falling
    asm volatile(
      "lds %A0, %3\n"
      "lds %B0, %4\n"
      "in %1, __SREG__\n"
      "sub %A0, %A2\n"
      "sbc %B0, %B2\n"
      "out __SREG__, %1\n"
        : "+r" (i_tmp), "+r" (i_sreg), "+r" (RxChannel4Start)
        : "i" (&TCNT1L), "i" (&TCNT1H));
    RxChannel4 = i_tmp;
  }
  asm volatile ("reti");
}
/*** END RECEIVER INTERRUPTS ***/


void receiverSetup()
{
  RX_ROLL_DIR   = INPUT;
  RX_PITCH_DIR  = INPUT;
  RX_COLL_DIR   = INPUT;
  RX_YAW_DIR    = INPUT;

  RX_ROLL   = 0;
  RX_PITCH  = 0;
  RX_COLL   = 0;
  RX_YAW    = 0;
  
  CenterRollValue = Config.CenterRollValue;
  CenterPitchValue = Config.CenterPitchValue;
  CenterCollValue = Config.CenterCollValue;
  CenterYawValue = Config.CenterYawValue;

  /*
   * timer1 (16bit) - run at 8MHz, used to measure Rx pulses
   * and to control ESC/servo pulse
   */
  TCCR1B = _BV(CS10);

  /*
   * Enable Rx pin interrupts
   */
  PCICR = _BV(PCIE0) | _BV(PCIE2);  // PCINT0..7, PCINT16..23 enable
  PCMSK0 = _BV(PCINT7);      // PB7
  PCMSK2 = _BV(PCINT17);      // PD1
  EICRA = _BV(ISC00) | _BV(ISC10);  // Any change INT0, INT1
  EIMSK = _BV(INT0) | _BV(INT1);    // External Interrupt Mask Register

}

/*
 * This adding 7 business is to emulate exactly a signed
 * divide at the zero point (-7 through 7 will become 0).
 */
int16_t fastdiv8(int16_t x) {
  if(x < 0)
    x+= 7;
  return x >> 3;
}

/*
 * Copy, scale, and offset the Rx inputs from the interrupt-modified
 * registers.
 *
 * If an intterupt occurs that updates an Rx variable here, SREG will be
 * copied to i_sreg. Bit 7 will not be set in that interrupt as it is
 * cleared by hardware, so i_sreg will not ever match 0xff. We use this
 * as a zero-cost flag to read again to avoid possibly-corrupted values.
 *
 * I could not find any way to avoid the optimizer killing the retry or
 * reordering it to be unsafe other than by doing the set in inline
 * assembler with a memory barrier.
 */
void RxGetChannels()
{
  uint8_t t = 0xff;
  do {
    asm volatile("mov %0, %1":"=r" (i_sreg),"=r" (t)::"memory");
    RxInRoll = fastdiv8(RxChannel1 - CenterRollValue * 8);
    RxInPitch = fastdiv8(RxChannel2 - CenterPitchValue * 8);
    RxInCollective = fastdiv8(RxChannel3 - 1120 * 8);
    RxInYaw = fastdiv8(RxChannel4 - CenterYawValue * 8);
  } while(i_sreg != t);
#ifdef TWIN_COPTER
  RxInOrgPitch = RxInPitch;
#endif
}

#ifndef SAVE_CENTER
void receiverStickCenter()
{
  uint8_t i;
  while(1) {
    RxGetChannels();
    i = abs(RxInRoll) + abs(RxInPitch) + abs(RxInYaw);
    LED = 1;
    while(i) {
      LED = 0;
      i--;
    }
  }
}
#else
void receiverStickCenter()
{
  uint8_t i;
  int16_t RawRoll  = 0;
  int16_t RawColl  = 0;
  int16_t RawYaw   = 0;
  int16_t RawPitch = 0;
  
  LED = 1;
  _delay_ms( 500 );

  for( i=0; i<8; i++ )
  {
      RxGetChannels();
      LED ^= 1;
      RawRoll  += RxChannel1/8;
      RawPitch += RxChannel2/8;
      RawColl  += RxChannel3/8;
      RawYaw   += RxChannel4/8;
      _delay_ms(100);
  }

  Config.CenterRollValue = RawRoll/8;
  Config.CenterPitchValue = RawPitch/8;
  Config.CenterCollValue = RawColl/8;
  Config.CenterYawValue = RawYaw/8;
  Save_Config_to_EEPROM();

  while(1) {
      LED ^= 1;
      _delay_ms( 1000 );
  }
}
#endif
