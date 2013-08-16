#include "ARCbus.h"
#include "ARCbus_internal.h"
#include <msp430.h>
#include "Magic.h"

RESET_ERROR saved_error;

void reset(unsigned char level,unsigned short source,int err, unsigned short argument){
  //disable interrupts
  __disable_interrupt();
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

//this function is called from startup code to set the structure values after main returns
//This is done to use symbolic values rather then hardcode values in asm
void main_return(void){
  saved_error.level=ERR_LEV_CRITICAL;
  saved_error.source=BUS_ERR_SRC_STARTUP;
  saved_error.err=STARTUP_ERR_MAIN_RETURN;
  saved_error.argument=0;
}

//called from startup code to check SFR for the reason for the reset
void startup_error_check(void){
  unsigned char flags=IFG1;
  IFG1=0;
  //check for watchdog reset
  if(flags&WDTIFG){
    saved_error.level=ERR_LEV_CRITICAL;
    saved_error.source=BUS_ERR_SRC_STARTUP;
    saved_error.err=STARTUP_ERR_WDT_RESET;
    saved_error.argument=0;
    //set magic value
    saved_error.magic=RESET_MAGIC_POST;
    return;
  }
  //check for power on reset
  if(flags&PORIFG){
    saved_error.level=BUS_ERR_LEV_ROUTINE_RST;
    saved_error.source=BUS_ERR_SRC_STARTUP;
    saved_error.err=STARTUP_ERR_POR;
    saved_error.argument=0;
    //set magic value
    saved_error.magic=RESET_MAGIC_POST;
    return;
  }
  //check for reset pin
  if(flags&RSTIFG){
    saved_error.level=BUS_ERR_LEV_ROUTINE_RST;
    saved_error.source=BUS_ERR_SRC_STARTUP;
    saved_error.err=STARTUP_ERR_RESET_PIN;
    saved_error.argument=0;
    //set magic value
    saved_error.magic=RESET_MAGIC_POST;
    return;
  }  
  //check for flash key violation
  if(FCTL3&KEYV){
    saved_error.level=ERR_LEV_CRITICAL;
    saved_error.source=BUS_ERR_SRC_STARTUP;
    saved_error.err=STARTUP_ERR_RESET_FLASH_KEYV;
    saved_error.argument=0;
    //set magic value
    saved_error.magic=RESET_MAGIC_POST;
    //clear KEYV flag
    FCTL3=FWKEY|LOCK;
    return;
  }
  //check for SVS reset
  if((SVSCTL&(SVSFG|PORON))==(SVSFG|PORON) && SVSCTL&(VLD0|VLD1|VLD2|VLD3)){
    saved_error.level=BUS_ERR_LEV_ROUTINE_RST;
    saved_error.source=BUS_ERR_SRC_STARTUP;
    saved_error.err=STARTUP_ERR_RESET_SVS;
    saved_error.argument=SVSCTL;
    //set magic value
    saved_error.magic=RESET_MAGIC_POST;
    //clear SVSFG flag
    SVSCTL&=~SVSFG;
    return;
  }
  //Unknown error encountered
  saved_error.level=ERR_LEV_CRITICAL;
  saved_error.source=BUS_ERR_SRC_STARTUP;
  saved_error.err=STARTUP_ERR_RESET_UNKNOWN;
  saved_error.argument=0;
  //set magic value
  saved_error.magic=RESET_MAGIC_POST;
}
