#include "ARCbus.h"
#include "ARCbus_internal.h"
#include <msp430.h>
#include "Magic.h"

RESET_ERROR saved_error;

void reset(unsigned char level,unsigned short source,int err, unsigned short argument){
  //disable interrupts
  ctl_global_interrupts_set(0);
  //set values in error struct
  saved_error.level=level;
  saved_error.source=source;
  saved_error.err=err;
  saved_error.argument=argument;
  //set magic value
  saved_error.magic=RESET_MAGIC_PRE;
  //cause WDT access violation by not writing WDT password
  WDTCTL=0;
  //TODO: code should never get here, report an error if it does
}
