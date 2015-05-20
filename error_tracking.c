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
  //code should never get here call the reset vector, we don't have many more options
  (*((void (**)(void))RESET_VECTOR))();
  //reset vector messes with the stack so it should never return here
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
  //check reset interrupt vector register
  //TODO: handle all errors with unique error codes
  switch(SYSRSTIV){
    case SYSRSTIV_NONE:        //No Interrupt pending
    break;
    case SYSRSTIV_BOR:         //BOR
      saved_error.level=BUS_ERR_LEV_ROUTINE_RST;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_POR;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    case SYSRSTIV_RSTNMI:      //RST/NMI pin
      saved_error.level=BUS_ERR_LEV_ROUTINE_RST;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_PIN;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    break;
    case SYSRSTIV_DOBOR:       //Software BOR
    break;
    case SYSRSTIV_LPM5WU:      //Port LPM5 Wake Up
    break;
    case SYSRSTIV_SECYV:       //Security violation
    break;
    case SYSRSTIV_SVSL:        //SVSL
    break;
    case SYSRSTIV_SVSH:        //SVSH
    break;
    case SYSRSTIV_SVML_OVP:    //SVML_OVP
    break;
    case SYSRSTIV_SVMH_OVP:    //SVMH_OVP
    break;
    case SYSRSTIV_DOPOR:       //Software POR
    break;
    case SYSRSTIV_WDTTO:       //WDT Time out
    //TODO : make these separate errors
    case SYSRSTIV_WDTKEY:      //WDTKEY violation
      saved_error.level=ERR_LEV_CRITICAL;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_WDT_RESET;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    case SYSRSTIV_KEYV:        //Flash Key violation
      saved_error.level=ERR_LEV_CRITICAL;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_FLASH_KEYV;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
      //clear KEYV flag
      FCTL3=FWKEY|LOCK;
    return;
    case SYSRSTIV_FLLUL:       //FLL unlock
    break;
    case SYSRSTIV_PERF:        //peripheral/config area fetch
    break;
    case SYSRSTIV_PMMKEY:      //PMMKEY violation
    break;

  }
  //Unknown error encountered
  saved_error.level=ERR_LEV_CRITICAL;
  saved_error.source=BUS_ERR_SRC_STARTUP;
  saved_error.err=STARTUP_ERR_RESET_UNKNOWN;
  saved_error.argument=0;
  //set magic value
  saved_error.magic=RESET_MAGIC_POST;
}
