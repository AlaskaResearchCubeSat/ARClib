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
    case ERR_TIME_INVALID:
      return "ERROR TICKER TIME IS NOT VALID";
    case ERR_TIME_TOO_OLD:
      return "ERROR TICKER TIME IS TOO OLD";
    case ERR_I2C_CLL:
      return "ERROR I2C Clock Low Timeout";
    case ERR_I2C_START_TIMEOUT:
      return "ERROR I2C start timeout";
    case ERR_I2C_TX_SELF:
      return "ERROR TX to self";
    case ERR_DMA_TIMEOUT:
      return "ERROR DMA timeout";
    //Error was not found
    default:
      return "UNKNOWN ERROR";
  }
}
