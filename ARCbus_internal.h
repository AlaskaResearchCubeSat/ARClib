#ifndef __ARC_BUS_INTERNAL_H
#define __ARC_BUS_INTERNAL_H
  #include <stddef.h>
  #include <Error.h>
  #include <ctl.h>
 
  #include "ARCbus.h"
  
  //define serial pins
  #define BUS_PIN_SDA       BIT1
  #define BUS_PIN_SCL       BIT0
  
  #define BUS_PINS_I2C      (BUS_PIN_SDA|BUS_PIN_SCL)
  
  #define BUS_PIN_SCK       BIT2
  #define BUS_PIN_SOMI      BIT3
  #define BUS_PIN_SIMO      BIT4
    
  #define BUS_PINS_SPI      (BUS_PIN_SOMI|BUS_PIN_SIMO|BUS_PIN_SCK)
  
  //ARCbus error sources
  enum{BUS_ERR_SRC_CTL=ERR_SRC_ARCBUS,BUS_ERR_SRC_MAIN_LOOP,BUS_ERR_SRC_STARTUP,BUS_ERR_SRC_ASYNC,BUS_ERR_SRC_SETUP,BUS_ERR_SRC_ALARMS,BUS_ERR_SRC_ERR_REQ};
  
  //error codes for CTL
  enum{CTL_ERR_HANDLER};
    
  //error codes for main loop
  enum{MAIN_LOOP_ERR_RESET,MAIN_LOOP_ERR_CMD_CRC,MAIN_LOOP_ERR_BAD_CMD,MAIN_LOOP_ERR_NACK_REC,MAIN_LOOP_ERR_SPI_COMPLETE_FAIL,
      MAIN_LOOP_ERR_SPI_CLEAR_FAIL,MAIN_LOOP_ERR_MUTIPLE_CDH,MAIN_LOOP_ERR_CDH_NOT_FOUND,MAIN_LOOP_ERR_RX_BUF_STAT,MAIN_LOOP_ERR_I2C_RX_BUSY,
      MAIN_LOOP_ERR_I2C_ARB_LOST,MAIN_LOOP_CDH_SUB_STAT_REC,MAIN_LOOP_RESET_FAIL,MAIN_LOOP_ERR_SVML,MAIN_LOOP_ERR_SVMH};
      
  //error codes for startup code
  enum{STARTUP_ERR_RESET_UNKNOWN,STARTUP_ERR_MAIN_RETURN,STARTUP_ERR_WDT_RESET,STARTUP_ERR_WDT_PW_RESET,STARTUP_ERR_BOR,STARTUP_ERR_RESET_PIN,STARTUP_ERR_RESET_FLASH_KEYV,
       STARTUP_ERR_RESET_SVSL,STARTUP_ERR_RESET_SVSH,STARTUP_ERR_RESET_FLLUL,STARTUP_ERR_RESET_PERF,STARTUP_ERR_RESET_PMMKEY,STARTUP_ERR_RESET_SECYV,STARTUP_ERR_RESET_INVALID,
       STARTUP_ERR_RESET_UNHANDLED,STARTUP_ERR_UNEXPECTED_DOBOR,STARTUP_ERR_UNEXPECTED_DOPOR,STARTUP_ERR_PMM_VCORE};
        
  //error codes for async
  enum{ASYNC_ERR_CLOSE_WRONG_ADDR,ASYNC_ERR_OPEN_ADDR,ASYNC_ERR_OPEN_BUSY,ASYNC_ERR_CLOSE_FAIL,ASYNC_ERR_DATA_FAIL};
          
  //error codes for setup 
  enum{SETUP_ERR_DCO_MISSING_CAL};
  
  //error codes for alarms
  enum{ALARMS_INVALID_TIME_UPDATE,ALARMS_REV_TIME_UPDATE,ALARMS_FWD_TIME_UPDATE,ALARMS_ADJ_TRIGGER};
      
  //error codes for error request
  enum{ERR_REQ_ERR_SPI_SEND,ERR_REQ_ERR_BUFFER_BUSY,ERR_REQ_ERR_MUTEX_TIMEOUT};
  
  #define BUS_ERR_LEV_ROUTINE_RST   (ERR_LEV_DEBUG+3)
  
  //flags for internal BUS events
  enum{BUS_INT_EV_I2C_CMD_RX=(1<<0),BUS_INT_EV_SPI_COMPLETE=(1<<1),BUS_INT_EV_BUFF_UNLOCK=(1<<2),BUS_INT_EV_RELEASE_MUTEX=(1<<3),BUS_INT_EV_I2C_RX_BUSY=(1<<4),BUS_INT_EV_I2C_ARB_LOST=(1<<5),BUS_INT_EV_SVML=(1<<6),BUS_INT_EV_SVMH=(1<<7)};

  //values for async setup command
  enum{ASYNC_OPEN,ASYNC_CLOSE};

  //all events for ARCBUS internal commands
  #define BUS_INT_EV_ALL    (BUS_INT_EV_I2C_CMD_RX|BUS_INT_EV_SPI_COMPLETE|BUS_INT_EV_BUFF_UNLOCK|BUS_INT_EV_RELEASE_MUTEX|BUS_INT_EV_I2C_RX_BUSY|BUS_INT_EV_I2C_ARB_LOST|BUS_INT_EV_SVML|BUS_INT_EV_SVMH)

  //flags for bus helper events
  enum{BUS_HELPER_EV_ASYNC_TIMEOUT=1<<0,BUS_HELPER_EV_SPI_COMPLETE_CMD=1<<1,BUS_HELPER_EV_SPI_CLEAR_CMD=1<<2,BUS_HELPER_EV_ASYNC_CLOSE=1<<3,BUS_HELPER_EV_ERR_REQ=1<<4,BUS_HELPER_EV_SUB_POWERUP=1<<5};
  
  //flags for I2C_PACKET structures
  enum{I2C_PACKET_STAT_EMPTY,I2C_PACKET_STAT_IN_PROGRESS,I2C_PACKET_STAT_COMPLETE};
  
  //size of I2C packet queue
  #define BUS_I2C_PACKET_QUEUE_LEN      5

  //all helper task events
  #define BUS_HELPER_EV_ALL (BUS_HELPER_EV_ASYNC_TIMEOUT|BUS_HELPER_EV_SPI_COMPLETE_CMD|BUS_HELPER_EV_SPI_CLEAR_CMD|BUS_HELPER_EV_ASYNC_CLOSE|BUS_HELPER_EV_ERR_REQ|BUS_HELPER_EV_SUB_POWERUP)
  
  //task structure for idle task and ARC bus task
  extern CTL_TASK_t idle_task,ARC_bus_task;
  
    //type for keeping track of errors
  typedef struct{
    int magic;
    unsigned short source;
    int err;
    unsigned short argument;
    unsigned char level;
  }RESET_ERROR;
  
  //structure for receiving I2C data
  typedef struct{
    unsigned char stat;
    unsigned char len;
    unsigned char dat[BUS_I2C_HDR_LEN+BUS_I2C_MAX_PACKET_LEN+BUS_I2C_CRC_LEN];
  }I2C_PACKET;

  extern RESET_ERROR saved_error;
  
  //stack for ARC bus task
  extern unsigned BUS_stack[256];
  
  extern BUS_STAT arcBus_stat;
  
  //buffer for ISR command receive
  extern I2C_PACKET I2C_rx_buf[BUS_I2C_PACKET_QUEUE_LEN];
  //queue indexes
  extern short I2C_rx_in,I2C_rx_out;
  
  //power status
  extern unsigned short powerState;
  
  
  //task structures
  extern CTL_TASK_t ARC_bus_task;
  
  //ticker time that the last time update happened at
  extern ticker last_time_update;

  //flag to see if time has been updated
  extern short timesync;
  
  //events for subsystems
  extern CTL_EVENT_SET_t SUB_events,BUS_helper_events,BUS_INT_events;

  //setup stuff for buffer usage
  void BUS_init_buffer(void);
  
  //address for async communications
  extern unsigned char async_addr;
  extern unsigned short async_timer;
  //queues for async communications
  extern CTL_BYTE_QUEUE_t async_txQ;
  extern CTL_BYTE_QUEUE_t async_rxQ;
  //close current connection
  int async_close_remote(void);
  //Open asynchronous when asked to by a board
  void async_open_remote(unsigned char addr);
  
  void BUS_I2C_release(void);
  
  void BUS_timer_timeout_check(void);
  //trigger alarms that may have been updated over
  void BUS_alarm_ticker_update(ticker newt,ticker oldt);

#endif
