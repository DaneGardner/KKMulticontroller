#ifndef CONFIG_H
#define CONFIG_H

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "typedefs.h"

#define SAVE_CENTER

#include "io_cfg.h"

/* Multicopter Type */
//#define SINGLE_COPTER
//#define DUAL_COPTER
//#define TWIN_COPTER
//#define TRI_COPTER
//#define QUAD_COPTER
#define QUAD_X_COPTER
//#define Y4_COPTER
//#define HEX_COPTER
//#define Y6_COPTER

#endif
