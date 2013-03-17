#include "config.h"
#include "gyros.h"
#include "settings.h"
#include "receiver.h"
#include "motors.h"
#include "soft_serial.h"

bool Armed;

static int16_t integral[3];          // PID integral term
static int16_t last_error[3];        // Last proportional error

static void setup(void);
static void loop(void);
int main(void);

static void setup()
{
  MCUCR = _BV(PUD);  // Disable hardware pull-up

  settingsSetup();
  receiverSetup();
  gyrosSetup();
  motorsSetup();
#ifdef SERIAL_PORT
  soft_serial( false );
  ss_begin(9600);

  ss_write_str( "\r\n\r\nConfig.RollGyroDirection   = " );
  ss_write_num( Config.RollGyroDirection );
  ss_write_str( "\r\nConfig.PitchGyroDirection  = " );
  ss_write_num( Config.PitchGyroDirection );
  ss_write_str( "\r\nConfig.YawGyroDirection    = " );
  ss_write_num( Config.YawGyroDirection );
  ss_write_str( "\r\nConfig.CenterRollValue     = " );
  ss_write_num( Config.CenterRollValue );
  ss_write_str( "\r\nConfig.CenterPitchValue    = " );
  ss_write_num( Config.CenterPitchValue );
  ss_write_str( "\r\nConfig.CenterCollValue     = " );
  ss_write_num( Config.CenterCollValue );
  ss_write_str( "\r\nConfig.CenterYawValue      = " );
  ss_write_num( Config.CenterYawValue );
  ss_write( '\r\n' );
#endif

  LED_DIR   = OUTPUT;
  LED    = 0;

  /*
   * This suits my ATmega88A: no Tx trim, output timings perfect.
   * See doc8271.pdf page ~401; beware the step at 128. -Simon
   */
  if(OSCCAL == 0x9d)
    OSCCAL = 0x9f;

  /*
   * timer2 8bit - run at 8MHz / 1024 = 7812.5KHz, just used for arming
   */
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);

  Armed = false;

  /*
   * Flash the LED once at power on
   */
  LED = 1;
  _delay_ms(250);
  LED = 0;

  sei();

  _delay_ms(2000);

  ReadGainPots();
  ReadGainPots();
#ifdef SERIAL_PORT
  ReadGyros();
  ss_write_str( "\r\nPitch gain   = " );
  ss_write_num( GainInADC[PITCH] );
  ss_write_str( "\r\nRoll gain    = " );
  ss_write_num( GainInADC[ROLL] );
  ss_write_str( "\r\nYaw gain     = " );
  ss_write_num( GainInADC[YAW] );
  ss_write_str( "\r\nPitch gyro   = " );
  ss_write_num( gyroADC[PITCH] );
  ss_write_str( "\r\nRoll gyro    = " );
  ss_write_num( gyroADC[ROLL] );
  ss_write_str( "\r\nYaw gyro     = " );
  ss_write_num( gyroADC[YAW] );
  ss_write( '\r\n' );
#endif

  bool pitchMin = (GainInADC[PITCH] < (ADC_MAX * 5) / 100);    // 5% threshold
  bool rollMin =  (GainInADC[ROLL]  < (ADC_MAX * 5) / 100);    // 5% threshold
  bool yawMin =   (GainInADC[YAW]   < (ADC_MAX * 5) / 100);    // 5% threshold

  if(pitchMin && rollMin && yawMin) { 
      #ifdef SERIAL_PORT
      ss_write_str( "\r\nClearing settings" );
      #endif
      settingsClearAll(); 
  }             // Clear config
  else if(pitchMin && yawMin)       { 
      #ifdef SERIAL_PORT
      ss_write_str( "\r\nMotor Identify" );
      #endif
      motorsIdentify(); 
  }               // Motor identification
//  else if(pitchMin && rollMin)      { }                                 // Future use
//  else if(rollMin && yawMin)        { }                                 // Future use
  else if(pitchMin)                 { 
      #ifdef SERIAL_PORT
      ss_write_str( "\r\nStick Center" );
      #endif
      receiverStickCenter(); 
  }          // Stick Centering Test
  else if(rollMin)                  { 
      #ifdef SERIAL_PORT
      ss_write_str( "\r\nGyro reverse" );
      #endif
      gyrosReverse(); 
  }                 // Gyro direction reversing
  else if(yawMin)                   { 
      #ifdef SERIAL_PORT
      ss_write_str( "\r\nThrottle Calibrate" );
      #endif
      motorsThrottleCalibration(); 
  }    // ESC throttle calibration
}

static inline void loop()
{
  static uint16_t Change_Arming = 0;
  static uint8_t Arming_TCNT2 = 0;
  int16_t error, emax = 1023;
  int16_t imax, derivative;

  RxGetChannels();

  if(RxInCollective <= 0) {
    // Check for stick arming (Timer2 at 8MHz/1024 = 7812.5KHz)
    Change_Arming += (uint8_t)(TCNT2 - Arming_TCNT2);
    Arming_TCNT2 = TCNT2;

    if(Armed) {
      if(RxInYaw < STICK_THROW || abs(RxInPitch) > STICK_THROW)
        Change_Arming = 0;    // re-set count
    } else {
      if(RxInYaw > -STICK_THROW || abs(RxInPitch) > STICK_THROW)
        Change_Arming = 0;    // re-set count
    }

    // 3Sec / 0.000128 = 23437 = 0x5B8D or
    // 2.5Sec / 0.000128 = 19531 = 0x4C4B
    // 0.5Sec / 0.000128 = 3906 = 0x0F42
    if(Change_Arming > 0x0F42) {
      Armed = !Armed;
      if(Armed)
        CalibrateGyros();
    }
  }

  ReadGyros();

  LED = Armed;

  gyroADC[ROLL]-= gyroZero[ROLL];
  gyroADC[PITCH]-= gyroZero[PITCH];
  gyroADC[YAW]-= gyroZero[YAW];

  //--- Start mixing by setting collective to motor outputs

  RxInCollective = (RxInCollective * 10) >> 3;  // 0-800 -> 0-1000

#ifndef SINGLE_COPTER
  if(RxInCollective > MAX_COLLECTIVE)
    RxInCollective = MAX_COLLECTIVE;
#endif

#ifdef SINGLE_COPTER
  MotorOut1 = RxInCollective;
  MotorOut2 = 840;  // 840
  MotorOut3 = 840;  // 840
  MotorOut4 = 945;  // 840 + 840/8
  MotorOut5 = 945;  // 840 + 840/8
#elif defined(DUAL_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = 500;
  MotorOut4 = 500;
#elif defined(TWIN_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = 500;
  MotorOut4 = 500;
  MotorOut5 = 500;  // Optional
  MotorOut6 = 500;  // Optional
#elif defined(TRI_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = RxInCollective;
  MotorOut4 = 500;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = RxInCollective;
  MotorOut4 = RxInCollective;
#elif defined(Y4_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = RxInCollective * 3 / 4;    // 25% Down
  MotorOut4 = RxInCollective * 3 / 4;    // 25% Down
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = RxInCollective;
  MotorOut4 = RxInCollective;
  MotorOut5 = RxInCollective;
  MotorOut6 = RxInCollective;
#endif

  imax = RxInCollective;
  if(imax < 0)
    imax = 0;
  imax>>= 3;  /* 1000 -> 200 */

  /* Calculate roll output - Test without props!! */

  RxInRoll = ((int32_t)RxInRoll * (uint32_t)GainInADC[ROLL]) >> STICK_GAIN_SHIFT;
  gyroADC[ROLL] = ((int32_t)gyroADC[ROLL] * (uint32_t)GainInADC[ROLL]) >> GYRO_GAIN_SHIFT;
  if(Config.RollGyroDirection == GYRO_NORMAL)
    gyroADC[ROLL] = -gyroADC[ROLL];

  if(Armed) {
    if(0) {
      error = RxInRoll - gyroADC[ROLL];
      if(error > emax)
        error = emax;
      else if(error < -emax)
        error = -emax;
      integral[ROLL]+= error;
      if(integral[ROLL] > imax)
        integral[ROLL] = imax;
      else if(integral[ROLL] < -imax)
        integral[ROLL] = -imax;
      derivative = error - last_error[ROLL];
      last_error[ROLL] = error;
      RxInRoll+= error + (integral[ROLL] >> 2) + (derivative >> 2);
    } else {
      RxInRoll-= gyroADC[ROLL];
    }
  }

#ifdef SINGLE_COPTER
  MotorOut2+= RxInRoll;
  MotorOut4-= RxInRoll;
#elif defined(DUAL_COPTER)
  MotorOut4+= RxInRoll;
#elif defined(TWIN_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;  // Approximation of sin(60) without div
  MotorOut1+= RxInRoll;
  MotorOut2-= RxInRoll;
#elif defined(TRI_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;  // (.875 versus .86602540)
  MotorOut1+= RxInRoll;
  MotorOut2-= RxInRoll;
#elif defined(QUAD_COPTER)
  MotorOut2+= RxInRoll;
  MotorOut3-= RxInRoll;
#elif defined(QUAD_X_COPTER)
  RxInRoll = RxInRoll >> 1;
  MotorOut1+= RxInRoll;
  MotorOut2-= RxInRoll;
  MotorOut3-= RxInRoll;
  MotorOut4+= RxInRoll;
#elif defined(Y4_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;
  MotorOut1+= RxInRoll;
  MotorOut2-= RxInRoll;
#elif defined(HEX_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;
  MotorOut2-= RxInRoll;
  MotorOut3-= RxInRoll;
  MotorOut5+= RxInRoll;
  MotorOut6+= RxInRoll;
#elif defined(Y6_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;
  MotorOut1+= RxInRoll;
  MotorOut2+= RxInRoll;
  MotorOut3-= RxInRoll;
  MotorOut4-= RxInRoll;
#endif

  /* Calculate pitch output - Test without props!! */

  RxInPitch = ((int32_t)RxInPitch * (uint32_t)GainInADC[PITCH]) >> STICK_GAIN_SHIFT;
  gyroADC[PITCH] = ((int32_t)gyroADC[PITCH] * (uint32_t)GainInADC[PITCH]) >> GYRO_GAIN_SHIFT;
  if(Config.PitchGyroDirection == GYRO_NORMAL)
    gyroADC[PITCH] = -gyroADC[PITCH];

  if(Armed) {
    if(0) {
      error = RxInPitch - gyroADC[PITCH];
      if(error > emax)
        error = emax;
      else if(error < -emax)
        error = -emax;
      integral[PITCH]+= error;
      if(integral[PITCH] > imax)
        integral[PITCH] = imax;
      else if(integral[PITCH] < -imax)
        integral[PITCH] = -imax;
      derivative = error - last_error[PITCH];
      last_error[PITCH] = error;
      RxInPitch+= error + (integral[PITCH] >> 2) + (derivative >> 2);
    } else {
      RxInPitch-= gyroADC[PITCH];
    }
  }

#ifdef SINGLE_COPTER
  MotorOut3+= RxInPitch;
  MotorOut5-= RxInPitch;
#elif defined(DUAL_COPTER)
  MotorOut3+= RxInPitch;
#elif defined(TWIN_COPTER)
  MotorOut3-= SERVO_REVERSE RxInPitch;
  MotorOut4+= SERVO_REVERSE RxInPitch;
  // Stick Only, Optional
  RxInOrgPitch = abs(RxInOrgPitch);
  MotorOut5+= RxInOrgPitch;      // Tain Servo-Optional, Down Only
  MotorOut6-= RxInOrgPitch;      // Tain Servo-Optional, Down Only (Reverse)
#elif defined(TRI_COPTER)
  MotorOut3-= RxInPitch;
  RxInPitch = (RxInPitch >> 1);      // cosine of 60
  MotorOut1+= RxInPitch;
  MotorOut2+= RxInPitch;
#elif defined(QUAD_COPTER)
  MotorOut1+= RxInPitch;
  MotorOut4-= RxInPitch;
#elif defined(QUAD_X_COPTER)
  RxInPitch = (RxInPitch >> 1);      // cosine of 60
  MotorOut1+= RxInPitch;
  MotorOut2+= RxInPitch;
  MotorOut3-= RxInPitch;
  MotorOut4-= RxInPitch;
#elif defined(Y4_COPTER)
  MotorOut1+= RxInPitch;
  MotorOut2+= RxInPitch;
  MotorOut3-= RxInPitch;
  MotorOut4-= RxInPitch;
#elif defined(HEX_COPTER)
  MotorOut1+= RxInPitch;
  MotorOut4-= RxInPitch;
  RxInPitch = (RxInPitch >> 2);
  MotorOut2+= RxInPitch;
  MotorOut3-= RxInPitch;
  MotorOut5-= RxInPitch;
  MotorOut6+= RxInPitch;
#elif defined(Y6_COPTER)
  MotorOut5-= RxInPitch;
  MotorOut6-= RxInPitch;
  RxInPitch = (RxInPitch >> 1);      // cosine of 60
  MotorOut1+= RxInPitch;
  MotorOut2+= RxInPitch;
  MotorOut3+= RxInPitch;
  MotorOut4+= RxInPitch;
#endif

  /* Calculate yaw output - Test without props!! */

  RxInYaw = ((int32_t)RxInYaw * (uint32_t)GainInADC[YAW]) >> STICK_GAIN_SHIFT;
  gyroADC[YAW] = ((int32_t)gyroADC[YAW] * (uint32_t)GainInADC[YAW]) >> GYRO_GAIN_SHIFT;
  if(Config.YawGyroDirection == GYRO_NORMAL)
    gyroADC[YAW] = -gyroADC[YAW];

  if(Armed) {
    error = RxInYaw - gyroADC[YAW];
    if(error > emax)
      error = emax;
    else if(error < -emax)
      error = -emax;
    integral[YAW]+= error;
    if(integral[YAW] > imax)
      integral[YAW] = imax;
    else if(integral[YAW] < -imax)
      integral[YAW] = -imax;
    derivative = error - last_error[YAW];
    last_error[YAW] = error;
    RxInYaw+= error + (integral[YAW] >> 4) + (derivative >> 4);
  }

#ifdef SINGLE_COPTER
  MotorOut2+= RxInYaw;
  MotorOut3+= RxInYaw;
  MotorOut4+= RxInYaw;
  MotorOut5+= RxInYaw;
#elif defined(DUAL_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut2+= RxInYaw;
#elif defined(TWIN_COPTER)
  MotorOut3+= SERVO_REVERSE(RxInYaw >> 1);
  MotorOut4+= SERVO_REVERSE(RxInYaw >> 1);
#elif defined(TRI_COPTER)
  MotorOut4+= SERVO_REVERSE RxInYaw;
#elif defined(QUAD_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut2+= RxInYaw;
  MotorOut3+= RxInYaw;
  MotorOut4-= RxInYaw;
#elif defined(QUAD_X_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut2+= RxInYaw;
  MotorOut3-= RxInYaw;
  MotorOut4+= RxInYaw;
#elif defined(Y4_COPTER)
  if((MotorOut3 - RxInYaw) < 100)
    RxInYaw = MotorOut3 - 100;  // Yaw Range Limit
  if((MotorOut3 - RxInYaw) > 1000)
    RxInYaw = MotorOut3 - 1000;  // Yaw Range Limit

  if((MotorOut4 + RxInYaw) < 100)
    RxInYaw = 100 - MotorOut4;  // Yaw Range Limit
  if((MotorOut4 + RxInYaw) > 1000)
    RxInYaw = 1000 - MotorOut4;  // Yaw Range Limit

  MotorOut3-= RxInYaw;
  MotorOut4+= RxInYaw;
#elif defined(HEX_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut2+= RxInYaw;
  MotorOut3-= RxInYaw;
  MotorOut4+= RxInYaw;
  MotorOut5-= RxInYaw;
  MotorOut6+= RxInYaw;
#elif defined(Y6_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut4-= RxInYaw;
  MotorOut5-= RxInYaw;
  MotorOut2+= RxInYaw;
  MotorOut3+= RxInYaw;
  MotorOut6+= RxInYaw;
#endif

#if defined(TRI_COPTER)
  /*
   * Rather than clipping the motor outputs and causing instability
   * at throttle saturation, we pull down the throttle of the other
   * motors. This gives priority to stabilization without a fixed
   * collective limit.
   */
  imax = MotorOut1;
  if(MotorOut2 > imax)
    imax = MotorOut2;
  if(MotorOut3 > imax)
    imax = MotorOut3;
  imax-= 1000;
  if(imax > 0) {
    MotorOut1-= imax;
    MotorOut2-= imax;
    MotorOut3-= imax;
  }
#endif

  imax = 114;
  //--- Limit the lowest value to avoid stopping of motor if motor value is under-saturated ---
  if(MotorOut1 < imax)
    MotorOut1 = imax;  // this is the motor idle level
  if(MotorOut2 < imax)
    MotorOut2 = imax;
#if defined(TRI_COPTER) || defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
  if(MotorOut3 < imax)
    MotorOut3 = imax;
#endif
#if defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
  if(MotorOut4 < imax)
    MotorOut4 = imax;
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
  if(MotorOut5 < imax)
    MotorOut5 = imax;
  if(MotorOut6 < imax)
    MotorOut6 = imax;
#endif

  //--- Output to motor ESC's ---
  if(RxInCollective < 1 || !Armed) {
    /* turn off motors unless armed and collective is non-zero */
#ifdef SINGLE_COPTER
    MotorOut1 = 0;
    MotorOut2 = 840;
    MotorOut3 = 840;
    MotorOut4 = 840;
    MotorOut5 = 840;
#elif defined(DUAL_COPTER)
    MotorOut1 = 0;
    MotorOut2 = 0;
    if(!Armed) {
      MotorOut3 = 500;
      MotorOut4 = 500;
    }
#elif defined(TWIN_COPTER)
    MotorOut1 = 0;
    MotorOut2 = 0;
    if(!Armed) {
      MotorOut3 = 500;
      MotorOut4 = 500;
      MotorOut5 = 500;
      MotorOut6 = 500;
    }
#elif defined(TRI_COPTER)

    MotorOut1 = 0;
    MotorOut2 = 0;
    MotorOut3 = 0;
    if(!Armed)
      MotorOut4 = 500;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
    MotorOut1 = 0;
    MotorOut2 = 0;
    MotorOut3 = 0;
    MotorOut4 = 0;
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
    MotorOut1 = 0;
    MotorOut2 = 0;
    MotorOut3 = 0;
    MotorOut4 = 0;
    MotorOut5 = 0;
    MotorOut6 = 0;
#endif
  }

  LED = 0;
  output_motor_ppm();

}


int main()
{
  setup();
#ifdef BT_SERIAL_PORT
  setLinvorRate( 115200 );
#endif

  while(1) {
    loop();
#ifdef SERIAL_PORT
	  menu();
#endif
  }
  return 1;
}
