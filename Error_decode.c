#include <Error.h>
#include "ARCbus_internal.h"

const char* cmdtostr(unsigned char cmd){
  switch(cmd){
    case CMD_NACK:
      return "Nack";
    case CMD_SPI_COMPLETE:
      return "SPI Complete";
    case CMD_SPI_RDY:
      return "SPI Ready";
    case CMD_SUB_ON:
      return "Subsystem On";
    case CMD_SUB_OFF:
      return "Subsystem Off";
    case CMD_SUB_POWERUP:
      return "Subsystem Powerup";
    case CMD_RESET:
      return "Reset";
    case CMD_SUB_STAT:
      return "Subsystem Stat";
    case CMD_SPI_CLEAR:
      return "SPI Clear";
    case CMD_EPS_STAT:
      return "EPS Status";
    case CMD_LEDL_STAT:
      return "LEDL Status";
    case CMD_ACDS_STAT:
      return "ACDS Status";
    case CMD_COMM_STAT:
      return "COMM Status";
    case CMD_IMG_STAT:
      return "IMG Status";
    case CMD_ASYNC_SETUP:
      return "Async Setup";
    case CMD_ASYNC_DAT:
      return "Async Data";
    default:
      return "Unknown";
  }
}

//decode errors from ACDS system
char *err_decode_arcbus(char buf[150], unsigned short source,int err, unsigned short argument){
  switch(source){
    case BUS_ERR_SRC_CTL:
      switch(err){
         case CTL_ERROR_NO_TASKS_TO_RUN: 
          return "CTL : No Tasks To Run";
        case CTL_UNSUPPORTED_CALL_FROM_ISR: 
          return "CTL : Unsupported Call From ISR";
        case CTL_UNSPECIFIED_ERROR:
          return "CTL : Unspecified Error";
        case CTL_MUTEX_UNLOCK_CALL_ERROR:
          return "CTL : Mutex Unlock Call Error";
        default:
          sprintf(buf,"CTL : unknown Error = %i, argument = %i",err,argument);
        return buf;
      }
    break;
    case BUS_ERR_SRC_MAIN_LOOP:
      switch(err){
        case MAIN_LOOP_ERR_RESET:
          return "ARCbus Main Loop : Commanded Reset";
        case MAIN_LOOP_ERR_CMD_CRC:
          sprintf(buf,"ARCbus Main Loop : bad CRC for command %s (%i)",cmdtostr(argument),argument);
        return buf;
      }
    break; 
    case BUS_ERR_SRC_STARTUP:
      switch(err){
        case STARTUP_ERR_MAIN_RETURN:
          return "Startup Code : Main Returned";
        case STARTUP_ERR_WDT_RESET:
          return "Startup Code : Watch Dog reset";
        case STARTUP_ERR_POR:
          return "Startup Code : Power On Reset";
        case STARTUP_ERR_RESET_PIN:
          return "Startup Code : Reset Pin Reset";
      }
    break; 
  }
  sprintf(buf,"source = %i, error = %i, argument = %i",source,err,argument);
  return buf;
}
