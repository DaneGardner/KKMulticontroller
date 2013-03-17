#ifndef RECEIVER_H
#define RECEIVER_H

#include "config.h"

/*** BEGIN DEFINES ***/
// Stick arming and throw detection (in % * 10 eg 1000 steps)
#define STICK_THROW 300

// Stick gain shift-right (after 32-bit multiplication of GainInADC[] value).
#define STICK_GAIN_SHIFT 8

// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilise better when full throttle applied
#define MAX_COLLECTIVE 1000      // 95

#define DEFAULT_CENTER 1520

/*** END DEFINES ***/


/*** BEGIN VARIABLES ***/
extern int16_t RxInRoll;
extern int16_t RxInPitch;
extern int16_t RxInCollective;
extern int16_t RxInYaw;

extern uint16_t RxChannel1;
extern uint16_t RxChannel2;
extern uint16_t RxChannel3;
extern uint16_t RxChannel4;

register uint16_t i_tmp asm("r2");               // ISR vars
register uint16_t RxChannel1Start asm("r4");
register uint16_t RxChannel2Start asm("r6");
register uint16_t RxChannel3Start asm("r8");
register uint16_t RxChannel4Start asm("r10");
register uint8_t i_sreg asm("r12");

#ifdef TWIN_COPTER
extern int16_t RxInOrgPitch;
#endif
/*** END VARIABLES ***/

/*** BEGIN PROTOTYPES ***/
ISR(PCINT2_vect, ISR_NAKED);
ISR(INT0_vect, ISR_NAKED);
ISR(INT1_vect, ISR_NAKED);
ISR(PCINT0_vect, ISR_NAKED);

void receiverSetup(void);
int16_t fastdiv8(int16_t x);
void RxGetChannels(void);
void receiverStickCenter(void);
/*** END PROTOTYPES ***/

#endif
