#ifndef TYPE_DEFS_H_
#define TYPE_DEFS_H_

#include <stdint.h>

#define INPUT 0
#define OUTPUT 1

typedef enum _BOOL { FALSE = 0, TRUE } BOOL;

// Code courtesy of: stu_san on AVR Freaks

typedef struct
{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg; 


#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt

#endif
