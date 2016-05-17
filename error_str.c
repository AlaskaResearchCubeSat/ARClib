#include "ARCbus.h"
#include "ARCbus_internal.h"

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

const char* BUS_cmdtostr(unsigned char cmd){
  switch(cmd){
    case CMD_NACK:
      return "CMD_NACK";
    case CMD_SPI_COMPLETE:
      return "CMD_SPI_COMPLETE";
    case CMD_SPI_RDY:
      return "CMD_SPI_RDY";
    case CMD_SUB_ON:
      return "CMD_SUB_ON";
    case CMD_SUB_OFF:
      return "CMD_SUB_OFF";
    case CMD_SUB_POWERUP:
      return "CMD_SUB_POWERUP";
    case CMD_RESET:
      return "CMD_RESET";
    case CMD_SUB_STAT:
      return "CMD_SUB_STAT";
    case CMD_SPI_CLEAR:
      return "CMD_SPI_CLEAR";
    case CMD_EPS_STAT:
      return "CMD_EPS_STAT";
    case CMD_LEDL_STAT:
      return "CMD_LEDL_STAT";
    case CMD_ACDS_STAT:
      return "CMD_ACDS_STAT";
    case CMD_COMM_STAT:
      return "CMD_COMM_STAT";
    case CMD_IMG_STAT:
      return "CMD_IMG_STAT";
    case CMD_ASYNC_SETUP:
      return "CMD_ASYNC_SETUP";
    case CMD_ASYNC_DAT:
      return "CMD_ASYNC_DAT";
    case CMD_SPI_DATA_ACTION:
      return "CMD_SPI_DATA_ACTION";
    case CMD_PING:
        return "CMD_PING";
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
    case CMD_BEACON_ON_OFF:
        return "CMD_BEACON_ON_OFF";
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
    case CMD_SPI_ABORT:
        return "CMD_SPI_ABORT";
    case CMD_BEACON_TYPE:
      return "CMD_BEACON_TYPE";
    case CMD_HW_RESET:
      return "CMD_HW_RESET";
    case CMD_RF_REQ:
      return "CMD_RF_REQ";
    default:
      return "Unknown";
  }
}

const char* BUS_cmd_resptostr(unsigned char resp){
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

const char* bus_flags_tostr(unsigned char flags){
  switch(flags){
    case BUS_FLAGS_INVALID_ADDR:
      return "BUS_FLAGS_INVALID_ADDR";
    case BUS_FLAGS_ADDR_DISABLED:
      return "BUS_FLAGS_ADDR_DISABLED";
    default:
      return "Unknown";
  }
}

const char * bus_version_err_tostr(signed char resp){
  switch(resp){
    case BUS_VER_SAME:
      return "BUS_VER_SAME";
    case BUS_VER_INVALID_MAJOR_REV:
      return "BUS_VER_INVALID_MAJOR_REV";
    case BUS_VER_MAJOR_REV_OLDER:
      return "BUS_VER_MAJOR_REV_OLDER";
    case BUS_VER_MAJOR_REV_NEWER:
      return "BUS_VER_MAJOR_REV_NEWER";
    case BUS_VER_INVALID_MINOR_REV:
      return "BUS_VER_INVALID_MINOR_REV";
    case BUS_VER_MINOR_REV_OLDER:
      return "BUS_VER_MINOR_REV_OLDER";
    case BUS_VER_MINOR_REV_NEWER:
      return "BUS_VER_MINOR_REV_NEWER";
    case BUS_VER_DIRTY_REV:
      return "BUS_VER_DIRTY_REV";
    case BUS_VER_HASH_MISMATCH:
      return "BUS_VER_HASH_MISMATCH";
    case BUS_VER_COMMIT_MISMATCH:
      return "BUS_VER_COMMIT_MISMATCH";
    default:
      return "Unknown";
  }
}
