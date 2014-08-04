#include "ARCbus.h"

//return error strings for error code
const char *BUS_error_str(int error){
  //check for error
  switch(error){
    case RET_SUCCESS:
      return "SUCCESS";
    case ERR_BAD_LEN:
      return "ERROR BAD LENGTH";
    case ERR_CMD_NACK:
      return "ERROR COMMAND NON-ACKNOWLEDGEMENT";
    case ERR_I2C_NACK:
      return "ERROR I2C NON-ACKNOWLEDGEMENT";
    case ERR_UNKNOWN:
      return "ERROR UNKNOWN";
    case ERR_BAD_ADDR:
      return "ERROR BAD ADDRESS";
    case ERR_BAD_CRC:
      return "ERROR BAD CRC";
    case ERR_TIMEOUT:
      return "ERROR TIMEOUT";
    case ERR_BUSY:
      return "ERROR DEVICE OR RESOURCE BUSY";
    case ERR_INVALID_ARGUMENT:
      return "ERROR INVALID ARGUMENT";
    case ERR_PACKET_TOO_LONG:
      return "ERROR PACKET TOO LONG";
    case ERR_I2C_ABORT:
      return "ERROR I2C Transaction Aborted";
    //Error was not found
    default:
      return "UNKNOWN ERROR";
  }
}
