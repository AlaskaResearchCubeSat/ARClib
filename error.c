#include <ctl.h>
#include <msp430.h>
#include "ARCbus.h"
#include "ARCbus_internal.h"
#include <Error.h>

//==============[task library error function]==============

//something went seriously wrong
//perhaps should try to recover/log error
void ctl_handle_error(CTL_ERROR_CODE_t e) __toplevel{
  //a really bad error occurred, reset and report the error 
  reset(ERR_LEV_CRITICAL,BUS_ERR_SRC_CTL,CTL_ERR_HANDLER,e);
}

