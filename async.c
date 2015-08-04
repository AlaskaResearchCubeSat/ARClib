#include <ctl.h>
#include <msp430.h>
#include <stdio.h>
#include "ARCbus.h"

#include "ARCbus_internal.h"

#define   ASYNC_TARGET_SIZE   (BUS_I2C_MAX_PACKET_LEN/2)
#define   ASYNC_MAX_SIZE      (BUS_I2C_MAX_PACKET_LEN)

unsigned char txbuf[256];
unsigned char rxbuf[300];

unsigned char async_addr=0;
unsigned short async_timer=0;

CTL_BYTE_QUEUE_t async_txQ;
CTL_BYTE_QUEUE_t async_rxQ;

CTL_EVENT_SET_t *closed_event=NULL;
CTL_EVENT_SET_t closed_flag=0;

//check if communicating with a board
int async_isOpen(void){
  return async_addr;
}

//Open asynchronous communications with a board
int async_open(unsigned char addr){
  int resp;
  unsigned char buff[BUS_I2C_HDR_LEN+1+BUS_I2C_CRC_LEN],*ptr;
  //check for general call address
  if(addr==BUS_ADDR_GC){
    //Error : can't open communication with GC address
    return ERR_BAD_ADDR;
  }
  //check for own address
  resp=OA_check(addr);
  if(resp!=RET_SUCCESS){
    //Error : can't open communication with own address
    return resp;
  }
  if(async_isOpen()){
    //Error: async is already open
    return ERR_BUSY;
  }
  //setup byte queues
  ctl_byte_queue_init(&async_txQ,txbuf,sizeof(txbuf));
  ctl_byte_queue_init(&async_rxQ,rxbuf,sizeof(rxbuf));
  //send command
  ptr=BUS_cmd_init(buff,CMD_ASYNC_SETUP);
  //send close command
  *ptr=ASYNC_OPEN;
  //send command
  resp=BUS_cmd_tx(addr,buff,1,0,BUS_I2C_SEND_FOREGROUND);
  //check for errors
  if(resp==RET_SUCCESS){
    //no errors, set address
    async_addr=addr;
  }
  return resp;
}

//Open asynchronous when asked to by a board
void async_open_remote(unsigned char addr){
  int resp;
  //check for general call address
  if(addr==BUS_ADDR_GC){
    //Error : can't open communication with GC address
    report_error(ERR_LEV_ERROR,BUS_ERR_SRC_ASYNC,ASYNC_ERR_OPEN_ADDR,addr);
    return;
  }
  //check for own address
  resp=OA_check(addr);
  if(resp!=RET_SUCCESS){
    //Error : can't open communication with own address
    report_error(ERR_LEV_ERROR,BUS_ERR_SRC_ASYNC,ASYNC_ERR_OPEN_ADDR,addr);
    return;
  }
  if(async_isOpen()){
    //Error: async is already open
    report_error(ERR_LEV_ERROR,BUS_ERR_SRC_ASYNC,ASYNC_ERR_OPEN_BUSY,(((unsigned short)addr)<<8)|async_addr);
    return;
  }
  //set address
  async_addr=addr;
  //setup byte queues
  ctl_byte_queue_init(&async_txQ,txbuf,sizeof(txbuf));
  ctl_byte_queue_init(&async_rxQ,rxbuf,sizeof(rxbuf));
  //send open event
  ctl_events_set_clear(&SUB_events,SUB_EV_ASYNC_OPEN,0);
}

//close current connection
int async_close(void){
  int resp,i;
  unsigned char buff[BUS_I2C_HDR_LEN+1+BUS_I2C_CRC_LEN],*ptr;
  if(!async_isOpen()){
    //async is not open, nothing to do
    return RET_SUCCESS;
  }
  //send remaining data
  async_send_data();
  //setup command
  ptr=BUS_cmd_init(buff,CMD_ASYNC_SETUP);
  //send close command
  *ptr=ASYNC_CLOSE;
  for(i=0;i<2;i++){
    //send command
    resp=BUS_cmd_tx(async_addr,buff,1,0,BUS_I2C_SEND_FOREGROUND);
    //check if command sent successfully
    if(resp==RET_SUCCESS){
      //clear address
      async_addr=0;
      //check for closed event
      if(closed_event){
        //send event
        ctl_events_set_clear(closed_event,closed_flag,0);
      }
      return resp;
    }else{
      //sending close command failed, report error
      report_error(ERR_LEV_ERROR,BUS_ERR_SRC_ASYNC,ASYNC_ERR_CLOSE_FAIL,resp);
    }
  }
  return resp;
}

//close current connection
int async_close_remote(void){
  //check if async is open
  if(!async_isOpen()){
    //Error: async is not open
    //TODO: better error?
    return ERR_BUSY;
  }
  //send remaining data
  async_send_data();
  //clear async address
  async_addr=0;
  //check for closed event
  if(closed_event){
    //send event
    ctl_events_set_clear(closed_event,closed_flag,0);
  }
  return RET_SUCCESS;
}

int async_send_data(void){
  unsigned char buff[BUS_I2C_HDR_LEN+ASYNC_MAX_SIZE+BUS_I2C_CRC_LEN];
  unsigned char *ptr;
  unsigned short len;
  int resp;
  //stop timer
  async_timer=0;
  //setup packet 
  ptr=BUS_cmd_init(buff,CMD_ASYNC_DAT);
  //get bytes from queue
  len=ctl_byte_queue_receive_multi(&async_txQ,ASYNC_MAX_SIZE,ptr,CTL_TIMEOUT_NOW,0);
  //check length
  if(len==0){
    return RET_SUCCESS;
  }
  //send data
  resp=BUS_cmd_tx(async_addr,buff,len,0,BUS_I2C_SEND_FOREGROUND);
  if(resp!=RET_SUCCESS){
    //sending data failed, report error
    report_error(ERR_LEV_ERROR,BUS_ERR_SRC_ASYNC,ASYNC_ERR_DATA_FAIL,resp);
  }
  //return result
  return resp;
}

//transmit a charecter
int async_TxChar(unsigned char c){
  unsigned int t;
  int res=c;
  //check if open
  if(!async_isOpen()){
    //Error: async is not open
    return EOF;
  }
  //queue byte
  ctl_byte_queue_post(&async_txQ,c,CTL_TIMEOUT_NONE,0);
  //check how many bytes are in the queue
  if(ctl_byte_queue_num_used(&async_txQ)>=ASYNC_TARGET_SIZE){
    //enough bytes to send now
    async_send_data();
  }else{
    //set timeout for next interval
    async_timer=30;
  }
  //return result
  return res;
}

int async_Getc(void){
  unsigned char c;
  //check if open
  if(!async_isOpen()){
    //Error: async is not open
    return EOF;
  }
  //receive a byte from the queue
  ctl_byte_queue_receive(&async_rxQ,&c,CTL_TIMEOUT_NONE,0);
  //return byte from queue
  return c;
}

int async_CheckKey(void){
  unsigned char c;
  if(ctl_byte_queue_receive_nb(&async_rxQ,&c)){
    return c;
  }else{
    return EOF;
  }
}

//setup events for byte queue
void async_setup_events(CTL_EVENT_SET_t *e,CTL_EVENT_SET_t txnotfull,CTL_EVENT_SET_t rxnotempty){
  ctl_byte_queue_setup_events(&async_rxQ,e,rxnotempty,0);
  ctl_byte_queue_setup_events(&async_txQ,e,0,txnotfull);
}

//setup closed event
void async_setup_close_event(CTL_EVENT_SET_t *e,CTL_EVENT_SET_t closed){
  closed_event=e;
  closed_flag=closed;
}
