#ifndef PID_H
#define PID_H

#include "config.h"

enum PID_ArrayIndex {
	PID_ROLL = 0,
	PID_PITCH,
	PID_YAW
};

struct PID_PidObject {
  float measured;

  float setPoint;
  float kP;
  float kI;
  float kD;
  float epsilon;

  uint64_t lastTime;
  float lastError;
  float integral;

  float pidOutput;
};

void PID_Calculate(struct PID_PidObject *pidObject);
void PID_CalculateAll(void);

#endif
