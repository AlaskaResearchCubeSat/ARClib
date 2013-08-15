#include <ctl.h>
#include <msp430.h>
#include "ARCbus.h"
#include "ARCbus_internal.h"
#include <Error.h>

//==============[task library error function]==============

//something went seriously wrong
//perhaps should try to recover/log error
void ctl_handle_error(CTL_ERROR_CODE_t e){
  switch(e){
    case CTL_ERROR_NO_TASKS_TO_RUN: 
      reset(ERR_LEV_CRITICAL,BUS_ERR_SRC_CTL,CTL_ERR_HANDLER,e);
    break;
    case CTL_UNSUPPORTED_CALL_FROM_ISR: 
      reset(ERR_LEV_CRITICAL,BUS_ERR_SRC_CTL,CTL_ERR_HANDLER,e);
    break;
    case CTL_UNSPECIFIED_ERROR:
      reset(ERR_LEV_CRITICAL,BUS_ERR_SRC_CTL,CTL_ERR_HANDLER,e);
    break;
    case CTL_MUTEX_UNLOCK_CALL_ERROR:
      reset(ERR_LEV_CRITICAL,BUS_ERR_SRC_CTL,CTL_ERR_HANDLER,e);
    break;
    default:
      reset(ERR_LEV_CRITICAL,BUS_ERR_SRC_CTL,CTL_ERR_HANDLER,e);
  }
}

