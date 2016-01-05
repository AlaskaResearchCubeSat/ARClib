#include "ARCbus.h"
#include "ARCbus_internal.h"
#include <msp430.h>
#include "Magic.h"

RESET_ERROR saved_error;

void reset_bor(unsigned char level,unsigned short source,int err, unsigned short argument){
  //disable interrupts
  __disable_interrupt();
  //set values in error struct
  saved_error.level=level;
  saved_error.source=source;
  saved_error.err=err;
  saved_error.argument=argument;
  //set magic value
  saved_error.magic=RESET_MAGIC_PRE;
  //cause a software Brown Out Reset to occur
  PMMCTL0=PMMPW|PMMSWBOR;
  //code should never get here call the reset vector, we don't have many more options
  (*((void (**)(void))RESET_VECTOR))();
  //reset vector messes with the stack so it should never return here
}

void reset_por(unsigned char level,unsigned short source,int err, unsigned short argument){
  //disable interrupts
  __disable_interrupt();
  //set values in error struct
  saved_error.level=level;
  saved_error.source=source;
  saved_error.err=err;
  saved_error.argument=argument;
  //set magic value
  saved_error.magic=RESET_MAGIC_PRE;
  //cause a software Power On Reset to occur
  PMMCTL0=PMMPW|PMMSWPOR;
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
  unsigned int sysrstiv_save;
  //save reset interrupt vector register
  sysrstiv_save=SYSRSTIV;
  //determine the reason for the reset
  switch(sysrstiv_save){
    case SYSRSTIV_NONE:        //No Interrupt pending
      //Unknown error encountered
      saved_error.level=ERR_LEV_CRITICAL;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_UNKNOWN;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    case SYSRSTIV_BOR:         //BOR
      saved_error.level=BUS_ERR_LEV_ROUTINE_RST;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_BOR;
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
    return;
    case SYSRSTIV_DOBOR:       //Software BOR
      if(RESET_MAGIC_PRE==saved_error.magic){
        //update magic value
        saved_error.magic=RESET_MAGIC_POST;
      }else{
        saved_error.level=ERR_LEV_ERROR;
        saved_error.source=BUS_ERR_SRC_STARTUP;
        saved_error.err=STARTUP_ERR_UNEXPECTED_DOBOR;
        saved_error.argument=0;
        //set magic value
        saved_error.magic=RESET_MAGIC_POST;
      }
    return;
    case SYSRSTIV_LPM5WU:      //Port LPM5 Wake Up
    break;
    case SYSRSTIV_SECYV:       //Security violation
      saved_error.level=ERR_LEV_CRITICAL;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_SECYV;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    case SYSRSTIV_SVSL:        //SVSL
      saved_error.level=BUS_ERR_LEV_ROUTINE_RST;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_SVSL;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    case SYSRSTIV_SVSH:        //SVSH
      saved_error.level=BUS_ERR_LEV_ROUTINE_RST;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_SVSH;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    case SYSRSTIV_SVML_OVP:    //SVML_OVP
    break;
    case SYSRSTIV_SVMH_OVP:    //SVMH_OVP
    break;
    case SYSRSTIV_DOPOR:       //Software POR
      if(RESET_MAGIC_PRE==saved_error.magic){
        //update magic value
        saved_error.magic=RESET_MAGIC_POST;
      }else{
        saved_error.level=ERR_LEV_ERROR;
        saved_error.source=BUS_ERR_SRC_STARTUP;
        saved_error.err=STARTUP_ERR_UNEXPECTED_DOPOR;
        saved_error.argument=0;
        //set magic value
        saved_error.magic=RESET_MAGIC_POST;
      }
    return;
    case SYSRSTIV_WDTTO:       //WDT Time out
      saved_error.level=ERR_LEV_ERROR;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_WDT_RESET;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    case SYSRSTIV_WDTKEY:      //WDTKEY violation
      saved_error.level=ERR_LEV_CRITICAL;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_WDT_PW_RESET;
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
      saved_error.level=ERR_LEV_CRITICAL;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_FLLUL;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    case SYSRSTIV_PERF:        //peripheral/config area fetch
      saved_error.level=ERR_LEV_CRITICAL;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_PERF;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    case SYSRSTIV_PMMKEY:      //PMMKEY violation
      saved_error.level=ERR_LEV_CRITICAL;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_PMMKEY;
      saved_error.argument=0;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
    default:
      //Unknown error encountered
      saved_error.level=ERR_LEV_CRITICAL;
      saved_error.source=BUS_ERR_SRC_STARTUP;
      saved_error.err=STARTUP_ERR_RESET_INVALID;
      saved_error.argument=sysrstiv_save;
      //set magic value
      saved_error.magic=RESET_MAGIC_POST;
    return;
  }
  //Unknown error encountered
  saved_error.level=ERR_LEV_CRITICAL;
  saved_error.source=BUS_ERR_SRC_STARTUP;
  saved_error.err=STARTUP_ERR_RESET_UNHANDLED;
  saved_error.argument=sysrstiv_save;
  //set magic value
  saved_error.magic=RESET_MAGIC_POST;
}
