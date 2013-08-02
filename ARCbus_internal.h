#ifndef __ARC_BUS_INTERNAL_H
#define __ARC_BUS_INTERNAL_H
  #include <stddef.h>
  //#define PRINT_DEBUG

  #ifdef PRINT_DEBUG
     //for testing
    #include <stdio.h>
  #endif
  
  //flags for internal BUS events
  enum{BUS_INT_EV_I2C_CMD_RX=(1<<0),BUS_INT_EV_SPI_COMPLETE=(1<<1),BUS_INT_EV_BUFF_UNLOCK=(1<<2),BUS_INT_EV_RELEASE_MUTEX=(1<<3)};

  //values for async setup command
  enum{ASYNC_OPEN,ASYNC_CLOSE};

  //all events for ARCBUS internal commands
  #define BUS_INT_EV_ALL    (BUS_INT_EV_I2C_CMD_RX|BUS_INT_EV_SPI_COMPLETE|BUS_INT_EV_BUFF_UNLOCK|BUS_INT_EV_RELEASE_MUTEX)

  //flags for bus helper events
  enum{BUS_HELPER_EV_ASYNC_TIMEOUT=1<<0,BUS_HELPER_EV_SPI_COMPLETE_CMD=1<<1,BUS_HELPER_EV_SPI_CLEAR_CMD=1<<2,BUS_HELPER_EV_ASYNC_CLOSE=1<<3};

  //all helper task events
  #define BUS_HELPER_EV_ALL (BUS_HELPER_EV_ASYNC_TIMEOUT|BUS_HELPER_EV_SPI_COMPLETE_CMD|BUS_HELPER_EV_SPI_CLEAR_CMD|BUS_HELPER_EV_ASYNC_CLOSE)
  
  //task structure for idle task and ARC bus task
  extern CTL_TASK_t idle_task,ARC_bus_task;
  
  //become master on the I2C bus and receive data
  short BUS_i2c_tx(unsigned short addr,const unsigned char *dat,unsigned short len);
  //become master on the I2C bus and transmit txLen bytes then recive rxlen bytes
  short BUS_i2c_txrx(unsigned short addr,const unsigned char *tx,unsigned short txLen,unsigned char *rx,unsigned short rxLen);
  
  //stack for ARC bus task
  extern unsigned BUS_stack[256];
  
  extern BUS_STAT arcBus_stat;
  
  //buffer for ISR command receive
  extern unsigned char i2c_buf[40];
  
  //power status
  extern unsigned short powerState=SUB_PWR_OFF;
  
  
  //task structures
  extern CTL_TASK_t ARC_bus_task;
  

  
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
  int async_open_remote(unsigned char addr);
  
  void BUS_I2C_release(void);

#endif
