#include "ARCbus.h"
#include <Error.h>
#include "ARCbus_internal.h"

//decode errors from ACDS system
const char *err_decode_arcbus(char buf[150], unsigned short source,int err, unsigned short argument){
  switch(source){
    case BUS_ERR_SRC_CTL:
      switch(err){
         case CTL_ERR_HANDLER:
           switch(argument){
             case CTL_ERROR_NO_TASKS_TO_RUN: 
              return "CTL : Error Handler : No Tasks To Run";
            case CTL_UNSUPPORTED_CALL_FROM_ISR: 
              return "CTL : Error Handler : Unsupported Call From ISR";
            case CTL_UNSPECIFIED_ERROR:
              return "CTL : Error Handler : Unspecified Error";
            case CTL_MUTEX_UNLOCK_CALL_ERROR:
              return "CTL : Error Handler : Mutex Unlock Call Error";
            default:
              sprintf(buf,"CTL : Error Handler : unknown error = %i",argument);
            return buf;
          }
         break;
      }
    break;
    case BUS_ERR_SRC_MAIN_LOOP:
      switch(err){
        case MAIN_LOOP_ERR_RESET:
          return "ARCbus Main Loop : Commanded Reset";
        case MAIN_LOOP_ERR_CMD_CRC:
          sprintf(buf,"ARCbus Main Loop : bad CRC for command %s (%i)",BUS_cmdtostr(argument),argument);
        return buf;
        case MAIN_LOOP_ERR_BAD_CMD:
          sprintf(buf,"ARCbus Main Loop : failed to parse command %s (%i), resp \"%s\" (%i)",BUS_cmdtostr(argument&0xFF),argument&0xFF,BUS_cmd_resptostr(argument>>8),argument>>8);
        return buf;
        case MAIN_LOOP_ERR_NACK_REC:
          sprintf(buf,"ARCbus Main Loop : NACK for command %s (%i), reason %i",BUS_cmdtostr(argument>>8),argument>>8,argument&0xFF);
        return buf;
        case MAIN_LOOP_ERR_SPI_COMPLETE_FAIL:
          sprintf(buf,"ARCbus Main Loop : Failed to send SPI complete command : %s",BUS_error_str(argument));
        return buf;
        case MAIN_LOOP_ERR_SPI_CLEAR_FAIL:
          sprintf(buf,"ARCbus Main Loop : Failed to send SPI clear command : %s",BUS_error_str(argument));
        return buf;
        case MAIN_LOOP_ERR_MUTIPLE_CDH:
          return "ARCbus Main Loop : Mutiple CDH boards detected!";
        case MAIN_LOOP_ERR_CDH_NOT_FOUND:
          sprintf(buf,"ARCbus Main Loop : CDH board not found : %s",BUS_error_str(argument));
        return buf;
        case MAIN_LOOP_ERR_RX_BUF_STAT:
          sprintf(buf,"ARCbus Main Loop : Incorrect I2C RX buffer status : %i. Ressetting I2C interface",argument);
          return buf;
        case MAIN_LOOP_ERR_I2C_RX_BUSY:
          return "ARCbus Main Loop : Rx Buffer busy, Packet Discarded";
        case MAIN_LOOP_ERR_I2C_ARB_LOST:
          return "ARCbus Main Loop : Arbitration Lost";
        case MAIN_LOOP_RESET_FAIL:
          return "ARCbus Main Loop : Reset Function Failed";
        case MAIN_LOOP_ERR_SVML:
          return "ARCbus Main Loop : Core Supply Low Error";
        case MAIN_LOOP_ERR_SVMH:
          return "ARCbus Main Loop : Input Supply Low Error";
        case MAIN_LOOP_SPI_ABORT:  
          sprintf(buf,"ARCbus Main Loop : Abort command recived, aborting SPI transaction. SPI addr = 0x%02X",argument);
        case MAIN_LOOP_ERR_SUBSYSTEM_VERSION_MISMATCH:
          sprintf(buf,"ARCbus Main Loop : Version mismatch for address 0x%02X : \"%s\" (%i)",(argument&0xFF),bus_version_err_tostr(argument>>8),(signed char)(argument>>8));
          return buf;
        case MAIN_LOOP_ERR_NACK_BUSY:
          sprintf(buf,"ARCbus Main Loop : NACK is busy failed to send NACK to 0x%02X : %s (%i)",(unsigned char)(argument>>8),BUS_cmd_resptostr(argument&0xFF),(argument&0xFF));
          return buf;
        case MAIN_LOOP_ERR_TX_NACK_FAIL:
          sprintf(buf,"ARCbus Main Loop : Failed to transmit NACK : %s (%i)",BUS_error_str(argument),argument);
          return buf;
        case MAIN_LOOP_ERR_UNEXPECTED_NACK_EV:
          return "ARCbus Main Loop : Unpected Tx NACK event";
      }
    break; 
    case BUS_ERR_SRC_STARTUP:
      switch(err){
        case STARTUP_ERR_MAIN_RETURN:
          return "Startup Code : Main Returned";
        case STARTUP_ERR_WDT_RESET:
          return "Startup Code : Watch Dog reset";
        case STARTUP_ERR_BOR:
          return "Startup Code : Power On Reset";
        case STARTUP_ERR_RESET_PIN:
          return "Startup Code : Reset Pin Reset";
        case STARTUP_ERR_RESET_FLASH_KEYV:
          return "Startup Code : Flash Security Key Violation";
        case STARTUP_ERR_RESET_UNKNOWN:
          return "Startup Code : Unknown Reset Cause";
        case STARTUP_ERR_RESET_SVSL:
          return "Startup Code : Supply Voltage Supervisor Reset (Low)";
        case STARTUP_ERR_RESET_SVSH:
          return "Startup Code : Supply Voltage Supervisor Reset (High)";
        case STARTUP_ERR_WDT_PW_RESET:
          return "Startup Code : Watch Dog Password Violation";
        case STARTUP_ERR_RESET_FLLUL:
          return "Startup Code : FLL lock lost";
        case STARTUP_ERR_RESET_PERF:
          return "Startup Code : Peripheral area fetch";
        case STARTUP_ERR_RESET_PMMKEY:
          return "Startup Code : PMM key violation";
        case STARTUP_ERR_RESET_SECYV:
          return "Startup Code : Security violation";
        case STARTUP_ERR_RESET_INVALID:
          sprintf(buf,"Startup Code : invalid reset code %02X",argument);
          return buf;
        case STARTUP_ERR_RESET_UNHANDLED:
          sprintf(buf,"Startup Code : unhandled reset code %02X",argument);
          return buf;
        case STARTUP_ERR_UNEXPECTED_DOBOR:
          return "Startup Code : unexpected software BOR";
        case STARTUP_ERR_UNEXPECTED_DOPOR:
          return "Startup Code : unexpected software POR";
        case STARTUP_ERR_PMM_VCORE:
          sprintf(buf,"Startup Code : failed to set Vcore PMMCTL0 = 0x%04X",argument);
          return buf;
        case STARTUP_LV_ERR_PMM_VCORE:
          sprintf(buf,"Startup Code : failed to set Vcore PMMCTL0 = 0x%04X",argument);
          return buf;
        case STARTUP_ERR_SVM_UNEXPECTED_VCORE:
          sprintf(buf,"Startup Code : failed to set SVM: unexpected Vcore PMMCTL0 = 0x%04X",argument);
          return buf;
        case STARTUP_ERR_NO_ERROR:
          return "Startup Code : Internal error, no stored startup error";
      }
    break; 
    case BUS_ERR_SRC_ASYNC:
      switch(err){
        case ASYNC_ERR_CLOSE_WRONG_ADDR:
          sprintf(buf,"Async : Wrong closing address. recived 0x%02X expected 0x%02X",argument>>8,argument&0xFF);
          return buf;
        case ASYNC_ERR_OPEN_ADDR:
          sprintf(buf,"Async : can't open addr 0x%02X",argument);
          return buf;
        case ASYNC_ERR_OPEN_BUSY:
          sprintf(buf,"Async : can't open async from addr 0x%02X already connected to 0x%02X",argument>>8,argument&0xFF);
          return buf;
        case ASYNC_ERR_CLOSE_FAIL:
          sprintf(buf,"Async : Failed to send closing command : %s",BUS_error_str(argument));
        return buf;
        case ASYNC_ERR_DATA_FAIL:
          sprintf(buf,"Async : Failed to send data : %s",BUS_error_str(argument));
        return buf;
      }
    break; 
    case BUS_ERR_SRC_SETUP:     
      switch(err){
        case SETUP_ERR_DCO_MISSING_CAL:
          return "ARClib Setup : Missing DCO Calibration Data";
      }
    break;
    case BUS_ERR_SRC_ALARMS:
        switch(err){
            case ALARMS_INVALID_TIME_UPDATE:
                sprintf(buf,"Alarms : Invalid time update, time diffrence %u",argument);
            return buf;
            case ALARMS_FWD_TIME_UPDATE:
                sprintf(buf,"Alarms : forward time update, time diffrence %u",argument);
            return buf;
            case ALARMS_REV_TIME_UPDATE:
                sprintf(buf,"Alarms : reverse time update, time diffrence %u",argument);
            return buf;
            case ALARMS_ADJ_TRIGGER:
                sprintf(buf,"Alarms : Alarm #%i was triggered due to time adjustment",argument);
            return buf;
        }
    break;
    case BUS_ERR_SRC_ERR_REQ:
        switch(err){
            case ERR_REQ_ERR_SPI_SEND:
              sprintf(buf,"Error Request : Failed to send data : %s",BUS_error_str(argument));
            return buf;
            case ERR_REQ_ERR_BUFFER_BUSY:
                return "Error Request : Buffer busy";
            case ERR_REQ_ERR_MUTEX_TIMEOUT:
                return "Error Request : Mutex lock timeout";
        }
    break;
    case BUS_ERR_SRC_I2C:
      switch(err){
        case I2C_ERR_INVALID_FLAGS:
            sprintf(buf,"I2C : Error, bad I2C flags (0x%02hhX) BUS_flags_to_addr returned %s (0x%02hhX)",(argument>>8),bus_flags_tostr(argument),argument);
        return buf;
        case I2C_ERR_TOO_MANY_ERRORS:
            sprintf(buf,"I2C : too many errors : %s (%i)",BUS_error_str(argument),argument);
        return buf;
      }
    break;
    case BUS_ERR_SRC_VERSION:
      switch(err){
        case VERSION_ERR_INVALID_MAJOR:
            sprintf(buf,"Version : Error invalid major revision. mine : %s, other : %s",(argument&VERSION_ERR_INVALID_MINE)?"invalid":"valid",(argument&VERSION_ERR_INVALID_OTHER)?"invalid":"valid");
        return buf;
        case VERSION_ERR_MAJOR_REV_NEWER:
            sprintf(buf,"Version : Error other major version is newer. version : %u",argument);
        return buf;
        case VERSION_ERR_MAJOR_REV_OLDER:
            sprintf(buf,"Version : Error other major version is older. version : %u",argument);
        return buf;
        case VERSION_ERR_INVALID_MINOR:
            sprintf(buf,"Version : Error invalid minor revision. mine : %s, other : %s",(argument&VERSION_ERR_INVALID_MINE)?"invalid":"valid",(argument&VERSION_ERR_INVALID_OTHER)?"invalid":"valid");
        return buf;
        case VERSION_ERR_MINOR_REV_NEWER:
            sprintf(buf,"Version : Error other minor version is newer. version : %u",argument);
        return buf;
        case VERSION_ERR_MINOR_REV_OLDER:
            sprintf(buf,"Version : Error other minor version is older. version : %u",argument);
        return buf;
        case VERSION_ERR_DIRTY_REV:
            sprintf(buf,"Version : dirty revision. mine : %s, other : %s",((argument>>8)==BUS_VER_CLEAN)?"clean":"dirty",((argument&&0x00FF)==BUS_VER_CLEAN)?"clean":"dirty");
        return buf;
        case VERSION_ERR_HASH_MISMATCH:
            return "Version : version hash mismatch";
        case VERSION_ERR_COMMIT_MISMATCH:
            sprintf(buf,"Version : commit number mismatch. other : %u",argument);
        return buf;
      }
    break;
  }
  sprintf(buf,"source = %i, error = %i, argument = %i",source,err,argument);
  return buf;
}
