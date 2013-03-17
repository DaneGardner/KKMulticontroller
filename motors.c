#include "motors.h"

#include "receiver.h"

int16_t MotorOut1;
int16_t MotorOut2;
int16_t MotorOut3;
int16_t MotorOut4;
int16_t MotorOut5;
int16_t MotorOut6;
int16_t MotorStartTCNT1;
#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
uint8_t servo_skip;
uint16_t servo_skip_divider;
#endif

void motorsSetup()
{
  M1_DIR    = OUTPUT;
  M2_DIR    = OUTPUT;
  M3_DIR    = OUTPUT;
  M4_DIR    = OUTPUT;
#ifdef SERIAL_PORT
  SER_TX_DIR = OUTPUT;
  SER_RX_DIR = INPUT;
#else
  M5_DIR    = OUTPUT;
  M6_DIR    = OUTPUT;
#endif

  /*
   * timer0 (8bit) - run at 8MHz, used to control ESC pulses
   * We use 8Mhz instead of 1MHz (1 usec) to avoid alignment jitter.
   */
  TCCR0B = _BV(CS00);  /* NOTE: Specified again below with FOC0x bits */

#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
  /*
   * Calculate the servo rate divider (pulse loop skip count
   * needed to avoid burning analog servos)
   */
  for(servo_skip_divider = 1;;servo_skip_divider++)
    if(servo_skip_divider * SERVO_RATE >= ESC_RATE)
      break;
#endif

}

void motorLoop()
{
}



void output_motor_ppm()
{
  int16_t t;

  /*
   * Bound pulse length to 1ms <= pulse <= 2ms.
   */

  t = 1000;
  if(MotorOut1 < 0)
    MotorOut1 = 0;
  else if(MotorOut1 > t)
    MotorOut1 = t;
#ifdef SINGLE_COPTER
  t = 2000;
#endif
  if(MotorOut2 < 0)
    MotorOut2 = 0;
  else if(MotorOut2 > t)
    MotorOut2 = t;
  if(MotorOut3 < 0)
    MotorOut3 = 0;
  else if(MotorOut3 > t)
    MotorOut3 = t;
  if(MotorOut4 < 0)
    MotorOut4 = 0;
  else if(MotorOut4 > t)
    MotorOut4 = t;
#ifndef SERIAL_PORT
  if(MotorOut5 < 0)
    MotorOut5 = 0;
  else if(MotorOut5 > t)
    MotorOut5 = t;
  if(MotorOut6 < 0)
    MotorOut6 = 0;
  else if(MotorOut6 > t)
    MotorOut6 = t;
#endif /* SERIAL_PORT */

  t = 1000;
  MotorOut1+= t;
#ifndef SINGLE_COPTER
  MotorOut2+= t;
  MotorOut3+= t;
  MotorOut4+= t;
#ifndef SERIAL_PORT
  MotorOut5+= t;
  MotorOut6+= t;
#endif /* SERIAL_PORT */
#endif

  MotorOut1<<= 3;
  MotorOut2<<= 3;
  MotorOut3<<= 3;
  MotorOut4<<= 3;
#ifndef SERIAL_PORT
  MotorOut5<<= 3;
  MotorOut6<<= 3;
#endif /* SERIAL_PORT */

  /*
   * Mirror M3, M4 to M5, M6, when possible, for hardware PPM
   * support. The compiler will throw away the above operations on
   * M5 and M6 when it sees these.
   */
#ifndef SERIAL_PORT
#if defined(DUAL_COPTER) || defined(TRI_COPTER) || defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
  MotorOut5 = MotorOut3;
  MotorOut6 = MotorOut4;
#endif
#endif /* SERIAL_PORT */

  /*
   * We can use timer compare output mode to provide jitter-free
   * PPM output on M1, M2, M5 and M6 by using OC0A and OC0B from
   * timer 0 (8-bit) and OC1A and OC1B from timer 1 (16-bit) to
   * turn off the pins. Since we are counting in steps of 1us and
   * need to wait up to 2ms, we need to delay the turn-on of the
   * 8-bit pins to avoid early triggering.
   *
   * Once entering compare match output mode, we cannot directly
   * set the pins. We can use the "force output compare" (which
   * doesn't actually force a compare but pretends the comparison
   * was true) to fiddle output high or low, but this would still
   * have interrupt and instruction-timing-induced jitter. Instead,
   * we just set the next desired switch state and set the OCRnx
   * registers to a known time in the future. The 8-bit ones will
   * set the pin the same way several times, so we have to make
   * sure that we don't change the high/low mode too early.
   *
   * Hardware PPM (timer compare output mode) pin mapping:
   *
   * M1 (PB2): OCR1B (COM1B) 16-bit
   * M2 (PB1): OCR1A (COM1A) 16-bit
   * M3 (PB0): software only
   * M4 (PD7): software only
   * M5 (PD6): OCR0A (COM0A) 8-bit
   * M6 (PD5): OCR0B (COM0B) 8-bit
   *
   * We must disable interrupts while setting the 16-bit registers
   * to avoid Rx interrupts clobbering the internal temporary
   * register for the associated 16-bit timer. 8 cycles is one
   * microsecond at 8 MHz, so we try not to leave interrupts
   * disabled for more than 8 cycles.
   *
   * We turn OFF the pins here, then wait for the ON cycle start.
   */
  t = MotorStartTCNT1 + MotorOut1;
  asm(""::"r" (t)); /* Avoid reordering of add after cli */
  cli();
  OCR1B = t;
  sei();
  t = MotorStartTCNT1 + MotorOut2;
  asm(""::"r" (t)); /* Avoid reordering of add after cli */
  cli();
  OCR1A = t;
  sei();
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);  /* Next match will clear pins */

  /*
   * Only 8 bits will make it to the OCR0x registers, so leave the
   * mode as setting pins ON here and then change to OFF mode after
   * the last wrap before the actual time.
   *
   * We hope that TCNT0 and TCNT1 are always synchronized.
   */
#ifdef SERIAL_PORT
  OCR0A = MotorStartTCNT1 + MotorOut3;
  OCR0B = MotorStartTCNT1 + MotorOut4;
#else
  OCR0A = MotorStartTCNT1 + MotorOut5;
  OCR0B = MotorStartTCNT1 + MotorOut6;
#endif

  do {
    cli();
    t = TCNT1;
    sei();
    t-= MotorStartTCNT1;
#ifdef SERIAL_PORT
    if(t + 0xff >= MotorOut3)
      TCCR0A&= ~_BV(COM0A0);  /* Clear pin on match */
    if(t + 0xff >= MotorOut4)
      TCCR0A&= ~_BV(COM0B0);  /* Clear pin on match */
#else
    if(t >= MotorOut3)
      M3 = 0;
    if(t >= MotorOut4)
      M4 = 0;
    if(t + 0xff >= MotorOut5)
      TCCR0A&= ~_BV(COM0A0);  /* Clear pin on match */
    if(t + 0xff >= MotorOut6)
      TCCR0A&= ~_BV(COM0B0);  /* Clear pin on match */
#endif
    t-= ((2000 + PWM_LOW_PULSE_US) << 3) - 0xff;
  } while(t < 0);

  /*
   * We should now be <= 0xff ticks before the next on cycle.
   *
   * Set up the timer compare values, wait for the on time, then
   * turn on software pins. We hope that we will be called again
   * within 1ms so that we can turn them off again in time.
   *
   * Timer compare output mode must stay enabled, and disables
   * regular output when enabled. The value of the COMnx0 bits set
   * the pin high or low when the timer value matches the OCRnx
   * value, or immediately when forced with the FOCnx bits.
   */

  MotorStartTCNT1+= (2000 + PWM_LOW_PULSE_US) << 3;
#if 0
  cli();
  t = TCNT1;
  sei();
  t+= 0x3f;
  t-= MotorStartTCNT1;
  if(t >= 0) {
    /*
     * We've already passed the on cycle, hmm.
     * Push it into the future.
     */
    cli();
    t = TCNT1;
    sei();
    MotorStartTCNT1 = t + 0xff;
  }
#endif
  t = MotorStartTCNT1;
  cli();
  OCR1B = t;
  sei();
  OCR0A = t;
  OCR0B = t;
  cli();
  OCR1A = t;
  sei();

#ifdef SINGLE_COPTER
  if(servo_skip == 0) {
    TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
//    TCCR1C = _BV(FOC1A) | _BV(FOC1B);
    TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
//    TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
  } else {
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0);
//    TCCR1C = _BV(FOC1A) | _BV(FOC1B);
  }
#elif defined(DUAL_COPTER) || defined(TWIN_COPTER)
  TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
//  TCCR1C = _BV(FOC1A) | _BV(FOC1B);
  if(servo_skip == 0) {
    TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
//    TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
  }
#elif defined(TRI_COPTER)
  TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
//  TCCR1C = _BV(FOC1A) | _BV(FOC1B);
  if(servo_skip == 0) {
    TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
//    TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
  } else {
    TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1);
//    TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
  }
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
  TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
//  TCCR1C = _BV(FOC1A) | _BV(FOC1B);
  TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
//  TCCR0B = _BV(CS00) | _BV(FOC0A) | _BV(FOC0B);
#endif

#ifndef SERIAL_PORT
  /*
   * Wait for the on time so we can turn on the software pins.
   */
  do {
    cli();
    t = TCNT1;
    sei();
    t-= MotorStartTCNT1;
  } while(t < 0);

#ifdef SINGLE_COPTER
  if(servo_skip == 0) {
    M3 = 1;
    M4 = 1;
    servo_skip = servo_skip_divider;
  }
  servo_skip--;
#elif defined(DUAL_COPTER) || defined(TWIN_COPTER)
  if(servo_skip == 0) {
    M3 = 1;
    M4 = 1;
    servo_skip = servo_skip_divider;
  }
  servo_skip--;
#elif defined(TRI_COPTER)
  M3 = 1;
  if(servo_skip == 0) {
    M4 = 1;
    servo_skip = servo_skip_divider;
  }
  servo_skip--;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER) || defined(HEX_COPTER) || defined(Y6_COPTER)
  M3 = 1;
  M4 = 1;
#endif
#endif
  /*
   * We leave with the output pins ON.
   */
}

void motorsIdentify()
{
  LED = 0;
  int8_t motor = 0;
  uint16_t delay = 0;
  uint16_t time = TCNT2;
  bool escInit = true;      // Wait until the ESCs have initialized

  while(1) {
    delay += (uint8_t)(TCNT2 - time);
    time = TCNT2;

    if(escInit) {
      if(delay > 23437) {    // 3.00 second delay (3.00 / .000128 = 23437.5)
        escInit = false;
        delay = 0;
      }
    } else if(LED) {
      if(delay > 1171) {    // 0.15 second delay (0.15 / .000128 = 1171.8)
        if(++motor > 6) {
          motor = 0;
        }
        delay = 0;
        LED = !LED;
      }
    } else {
      if(delay > 7812) {    // 1.00 second delay (1.00 / .000128 = 7812.5)
        delay = 0;
        LED = !LED;
      }
    }

    MotorOut1 = 0;
    MotorOut2 = 0;
    MotorOut3 = 0;
    MotorOut4 = 0;
    MotorOut5 = 0;
    MotorOut6 = 0;

    if(LED) {
      if(motor == 1) { MotorOut1 = 50; }
      if(motor == 2) { MotorOut2 = 50; }
      if(motor == 3) { MotorOut3 = 50; }
      if(motor == 4) { MotorOut4 = 50; }
      if(motor == 5) { MotorOut5 = 50; }
      if(motor == 6) { MotorOut6 = 50; }
    }

    output_motor_ppm();
  }
}

void motorsThrottleCalibration()
{
  // flash LED 3 times
  for(uint8_t i = 0;i < 3;i++) {
    LED = 1;
    _delay_ms(25);
    LED = 0;
    _delay_ms(25);
  }

  while(1) {
    RxGetChannels();
#ifdef SINGLE_COPTER
    MotorOut1 = RxInCollective;
    MotorOut2 = 1400;    // Center: 140
    MotorOut3 = 1400;
    MotorOut4 = 1400;
    MotorOut5 = 1400;
#elif defined(DUAL_COPTER)
    MotorOut1 = RxInCollective;
    MotorOut2 = RxInCollective;
    MotorOut3 = 500;    // Center: 50
    MotorOut4 = 500;
#elif defined(TWIN_COPTER)
    MotorOut1 = RxInCollective;
    MotorOut2 = RxInCollective;
    MotorOut3 = 500;    // Center: 50
    MotorOut4 = 500;
    MotorOut5 = 500;
    MotorOut6 = 500;    // Center: 50, Reverse
#elif defined(TRI_COPTER)
    MotorOut1 = RxInCollective;
    MotorOut2 = RxInCollective;
    MotorOut3 = RxInCollective;
    MotorOut4 = 500+RxInYaw*2;    // Center: 50
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
    MotorOut1 = RxInCollective;
    MotorOut2 = RxInCollective;
    MotorOut3 = RxInCollective;
    MotorOut4 = RxInCollective;
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
    MotorOut1 = RxInCollective;
    MotorOut2 = RxInCollective;
    MotorOut3 = RxInCollective;
    MotorOut4 = RxInCollective;
    MotorOut5 = RxInCollective;
    MotorOut6 = RxInCollective;
#else
#error No Copter configuration defined !!!!
#endif
    output_motor_ppm();  // this regulates rate at which we output signals
  }
}
