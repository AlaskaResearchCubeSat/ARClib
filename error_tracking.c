#include "ARCbus.h"
#include "ARCbus_internal.h"
#include <msp430.h>

void reset(unsigned char level,unsigned short source,int err, unsigned short argument){

  WDTCTL=0;
  //TODO: code should never get here, report an error if it does
}
