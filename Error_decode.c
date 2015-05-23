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
    case CMD_SPI_DATA_ACTION:
      return "SPI Data Action";
    case CMD_PING:
        return "CMD_PING";
    case CMD_MAG_DATA:
        return "CMD_MAG_DATA";
    case CMD_MAG_SAMPLE_CONFIG:
        return "CMD_MAG_SAMPLE_CONFIG";
    case CMD_ERR_REQ:
        return "CMD_ERR_REQ";
    case CMD_IMG_READ_PIC:
        return "CMD_IMG_READ_PIC";
    case CMD_IMG_TAKE_TIMED_PIC:
        return "CMD_IMG_TAKE_TIMED_PIC";
    case CMD_IMG_TAKE_PIC_NOW:
        return "CMD_IMG_TAKE_PIC_NOW";
    case CMD_GS_DATA:
        return "CMD_GS_DATA";
    case CMD_TEST_MODE:
        return "CMD_TEST_MODE";
    case CMD_BEACON_ON:
        return "CMD_BEACON_ON";
    case CMD_ACDS_CONFIG:
        return "CMD_ACDS_CONFIG";
    case CMD_IMG_CLEARPIC:
        return "CMD_IMG_CLEARPIC";
    case CMD_LEDL_READ_BLOCK:
        return "CMD_LEDL_READ_BLOCK";
    case CMD_ACDS_READ_BLOCK:
        return "CMD_ACDS_READ_BLOCK";
    case CMD_EPS_SEND:
        return "CMD_EPS_SEND";
    case CMD_LEDL_BLOW_FUSE:
        return "CMD_LEDL_BLOW_FUSE";
    default:
      return "Unknown";
  }
}

const char* cmd_resptostr(unsigned char resp){
  switch(resp){
    case RET_SUCCESS:
      return "Success";
    case ERR_PK_LEN:
      return "Error Invalid Packet Length";
    case ERR_UNKNOWN_CMD:
      return "Error Unknown Command";
    case ERR_SPI_LEN:
      return "Error SPI Block Length";
    case ERR_BAD_PK:
      return "Error Bad Packet";
    case ERR_SPI_BUSY:
      return "Error SPI busy";
    case ERR_BUFFER_BUSY:
      return "Error Buffer Busy";
    case ERR_ILLEAGLE_COMMAND:
      return "Error Illeagle Command";
    case ERR_SPI_NOT_RUNNING:
      return "Error SPI not running";
    case ERR_SPI_WRONG_ADDR:
      return "Error SPI wrong address";
    case ERR_PK_BAD_PARM:
      return "Error Bad parameter";
    default:
      return "Unknown";
  }
}

//decode errors from ACDS system
char *err_decode_arcbus(char buf[150], unsigned short source,int err, unsigned short argument){
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
          sprintf(buf,"ARCbus Main Loop : bad CRC for command %s (%i)",cmdtostr(argument),argument);
        return buf;
        case MAIN_LOOP_ERR_BAD_CMD:
          sprintf(buf,"ARCbus Main Loop : failed to parse command %s (%i), resp \"%s\" (%i)",cmdtostr(argument&0xFF),argument&0xFF,cmd_resptostr(argument>>8),argument>>8);
        return buf;
        case MAIN_LOOP_ERR_NACK_REC:
          sprintf(buf,"ARCbus Main Loop : NACK for command %s (%i), reason %i",cmdtostr(argument>>8),argument>>8,argument&0xFF);
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
  }
  sprintf(buf,"source = %i, error = %i, argument = %i",source,err,argument);
  return buf;
}
