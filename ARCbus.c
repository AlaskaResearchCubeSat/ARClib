#include <ctl.h>
#include <msp430.h>
#include <stdlib.h>
#include "timerA.h"
#include "ARCbus.h"
#include "crc.h"
#include "spi.h"

#include "ARCbus_internal.h"




//=======================================================================================
//                                 [BUS Functions]
//=======================================================================================

//return own address
unsigned char BUS_get_OA(void){
  int i;
  //base address for own I2C addresses
  volatile unsigned int * const oa_base=&UCB0I2COA0;
  //loop through addresses and return the first one that matches
  //TODO: perhaps there is a better way to do this (on a per task basis?)
  for(i=0;i<4;i++){
    //check if address enabled
    if(oa_base[i]&UCOAEN){
      //return address
      return (~(UCGCEN|UCOAEN))&oa_base[i];
    }
  }
  //no match! return OA0
  //TODO: report error? do something else?
  return UCB0I2COA0;
}

//Setup buffer for command 
unsigned char *BUS_cmd_init(unsigned char *buf,unsigned char id){
  buf[1]=id;
  //set originator address
  buf[0]=BUS_get_OA();
  //start of payload
  return buf+2;
}

//check for own address
int BUS_OA_check(unsigned char addr){
  int i;
  //base address for own I2C addresses
  volatile unsigned int * const oa_base=&UCB0I2COA0;
  //check if addr matches any slave address
  for(i=0;i<4;i++){
    //skip if address disabled
    if(!(oa_base[i]&UCOAEN))continue;
    //check for address match
    if(addr==((~(UCGCEN|UCOAEN))&oa_base[i])){
      return ERR_BAD_ADDR;
    }
  }
  //not a match success!
  return RET_SUCCESS;
}

//function to check I2C addresses
int addr_chk(unsigned char addr){
  //check if 8th bit is set
  if(addr&0x80){
    return ERR_BAD_ADDR;
  }
  //success!
  return RET_SUCCESS;
}

static ticker packet_time=0;

static unsigned BUS_I2C_lock(void){
  int i;
  //try to capture mutex
  if(0==ctl_mutex_lock(&arcBus_stat.i2c_stat.mutex,CTL_TIMEOUT_DELAY,10)){
     return ERR_BUSY;
  }
  return 0;
} 

//release the I2C bus
void BUS_I2C_release(void){
  ctl_mutex_unlock(&arcBus_stat.i2c_stat.mutex);
}

//send command
int BUS_cmd_tx(unsigned char addr,void *buff,unsigned short len,unsigned short flags,short bgnd){
  unsigned int e;
  short ret;
  int i;
  int mutex_release;
  unsigned char resp[2];
  //check address
  if((ret=addr_chk(addr))!=RET_SUCCESS){
    //return error if it occured
    return ret;
  }
  //check packet length
  if(len>BUS_I2C_MAX_PACKET_LEN){
    return ERR_PACKET_TOO_LONG;
  }
  //add standard header length
  len+=BUS_I2C_HDR_LEN;
  //add NACK flag if requested
  if(flags&BUS_CMD_FL_NACK){
    //request NACK
    ((unsigned char*)buff)[0]|=CMD_TX_NACK;
  }else{
    //clear NACK request
    ((unsigned char*)buff)[0]&=~CMD_TX_NACK;
  }
  //calculate CRC
  ((unsigned char*)buff)[len]=crc7(buff,len);
  //add a byte for the CRC
  len+=BUS_I2C_CRC_LEN;
  //check for zero length
  if(len==0){
    return ERR_BAD_LEN;
  }
  //if running in the background check to see that we are the ARCbus task
  if(bgnd){
    if(ctl_task_executing!=&ARC_bus_task){
      //invalid argument
      return ERR_INVALID_ARGUMENT;
    }
    //release mutex after complete
    mutex_release=1;
  }else{
    //don't release mutex after complete
    mutex_release=0;
  }
  //wait for the bus to become free
  if(BUS_I2C_lock()){
    //I2C bus is in use
    return ERR_BUSY;
  }
  //only change mutex_relase in arcBus structure after lock is obtained
  arcBus_stat.i2c_stat.mutex_release=mutex_release;
  //make sure that we are not calling while running in the background
  if(ctl_task_executing!=&ARC_bus_task && arcBus_stat.i2c_stat.mutex.lock_count!=1){
    //only allow function to be entered once at a time
    //release I2C bus
    BUS_I2C_release();
    //TODO : perhaps provide a better error here
    return ERR_BUSY;
  }
  //Setup for I2C transaction  
  //set slave address
  UCB0I2CSA=addr;
  //set index
  arcBus_stat.i2c_stat.tx.idx=0;
  //set I2C master state
  arcBus_stat.i2c_stat.tx.stat=BUS_I2C_MASTER_PENDING;
  //set length
  arcBus_stat.i2c_stat.tx.len=len;
  //set data
  arcBus_stat.i2c_stat.tx.ptr=(unsigned char*)buff;
  //set to transmit mode
  UCB0CTLW0|=UCTR;
  //clear master I2C flags
  ctl_events_set_clear(&arcBus_stat.events,0,BUS_EV_I2C_MASTER|BUS_EV_I2C_MASTER_START);
  //set master mode
  UCB0CTLW0|=UCMST;
  //UCB0CTLW0|=UCMST|UCTR;
  //generate start condition
  UCB0CTL1|=UCTXSTT;
  //if transmitting in the background, return
  if(bgnd){
    //return success
    return RET_SUCCESS;
  }
  //wait for packet to start
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&arcBus_stat.events,BUS_EV_I2C_MASTER_START,CTL_TIMEOUT_DELAY,50);
  //check to see if there was a problem
  if(!(e&BUS_EV_I2C_MASTER_STARTED)){
    //clear start bit
    UCB0CTL1&=~UCTXSTT;
    //release I2C bus
    BUS_I2C_release();
    //set I2C master state
    arcBus_stat.i2c_stat.tx.stat=BUS_I2C_MASTER_IDLE;
    //chech which error happened
    switch(e&BUS_EV_I2C_MASTER_START){
      case 0:
        //no event happened so timeout
        return ERR_I2C_START_TIMEOUT;
      case BUS_EV_I2C_NACK:
        //I2C device did not acknowledge
        return ERR_I2C_NACK;
      default:
        //error is not defined
        return ERR_UNKNOWN;
    }
  }
  //wait for transaction to complete
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&arcBus_stat.events,BUS_EV_I2C_MASTER,CTL_TIMEOUT_DELAY,50);
  //save transaction time
  packet_time=get_ticker_time();
  //release I2C bus
  BUS_I2C_release();
  //set I2C master state
  arcBus_stat.i2c_stat.tx.stat=BUS_I2C_MASTER_IDLE;
  //check which event(s) happened
  switch(e&BUS_EV_I2C_MASTER){
    case BUS_EV_I2C_COMPLETE:
      //no error
      return RET_SUCCESS;
    case BUS_EV_I2C_NACK:
      //I2C device did not acknowledge
      return ERR_I2C_NACK;
    case BUS_EV_I2C_ABORT:
      //I2C device did not acknowledge
      return ERR_I2C_ABORT;
    case 0:
      //no event happened, so time out
      return ERR_TIMEOUT;
    case BUS_EV_I2C_ERR_CCL:
      //Clock low timeout
      return ERR_I2C_CLL;
    case BUS_EV_I2C_TX_SELF:
      //TX to self and no one else responded
      return ERR_I2C_TX_SELF;
    default:
      //error is not defined
      return ERR_UNKNOWN;
  }
}

//send/receive SPI data over the bus
int BUS_SPI_txrx(unsigned char addr,void *tx,void *rx,unsigned short len){
  unsigned char buf[10],*ptr;
  unsigned int e;
  short time;
  int i,resp;
  unsigned short crc;
  //check address
  if((resp=addr_chk(addr))!=RET_SUCCESS){
    //return error if it occured
    return resp;
  }
  //reject own address
  if((resp=BUS_OA_check(addr))!=RET_SUCCESS){
    //return error if it occured
    return resp;
  }
  //reject General call address
  if(addr==BUS_ADDR_GC){
    return ERR_BAD_ADDR;
  }
  //calculate CRC
  crc=crc16(tx,len);
  //send CRC in Big endian order
  ((unsigned char*)tx)[len]=crc>>8;
  ((unsigned char*)tx)[len+1]=crc;
  //setup SPI structure
  arcBus_stat.spi_stat.len=len;
  arcBus_stat.spi_stat.rx=rx;
  arcBus_stat.spi_stat.tx=tx;
  arcBus_stat.spi_stat.nack=0;
  //disable DMA
  DMA0CTL&=~DMAEN;
  DMA1CTL&=~DMAEN;
  //Setup SPI
  SPI_slave_setup();
  //setup DMA for transfer
  DMACTL0 &=~(DMA0TSEL_15|DMA1TSEL_15);
  DMACTL0 |= (DMA0TSEL__USCIA0RX|DMA1TSEL__USCIA0TX);
  //====[DMA channel0 used for receive]====
  //check for omitted receive buffer
  if(rx!=NULL){
    // Source DMA address: receive register.
    *((unsigned int*)&DMA0SA) = (unsigned short)(&UCA0RXBUF);
    // Destination DMA address: rx buffer.
    *((unsigned int*)&DMA0DA) = (unsigned short)rx;
    // The size of the block to be transferred
    DMA0SZ = len+BUS_SPI_CRC_LEN;
    // Configure the DMA transfer, single byte transfer with source increment
    DMA0CTL =DMADT_0|DMASBDB|DMAEN|DMADSTINCR1|DMADSTINCR0;
  }
  //====[DMA channel1 used for transmit]====
  // Destination DMA address: the transmit buffer.
  *((unsigned int*)&DMA1DA) = (unsigned int)(&UCA0TXBUF);
  //check for omitted transmit buffer
  if(tx!=NULL){
    // Source DMA address: tx buffer
    *((unsigned int*)&DMA1SA) =((unsigned int)tx)+1;
    // The size of the block to be transferred
    DMA1SZ = len+BUS_SPI_CRC_LEN-1;
    // Configure the DMA transfer, single byte transfer with destination increment
    //enable interrupt to notify code when transfer is complete
    DMA1CTL=DMADT_0|DMASBDB|DMASRCINCR1|DMASRCINCR0|DMAEN;
    //start things off with an initial transfer
    UCA0TXBUF=*((unsigned char*)tx);
  }else{
    //need to send something to receive something so setup TX for dummy bytes
    *((unsigned int*)&DMA1SA) = (unsigned int)(&UCA0TXBUF);
    // The size of the block to be transferred
    DMA1SZ = len+BUS_SPI_CRC_LEN-1;
    // Configure the DMA transfer, single byte transfer with no increment
    DMA1CTL=DMADT_0|DMASBDB|DMASRCINCR0|DMASRCINCR0|DMAEN;
    //start things off with an initial transfer
    UCA0TXBUF=BUS_SPI_DUMMY_DATA;
  }
  //send SPI setup command
  ptr=BUS_cmd_init(buf,CMD_SPI_RDY);
  //send MSB first
  ptr[0]=len>>8;
  //then send LSB
  ptr[1]=len;
  //send command
  resp=BUS_cmd_tx(addr,buf,2,BUS_CMD_FL_NACK,BUS_I2C_SEND_FOREGROUND);
  //check if sent correctly
  if(resp!=RET_SUCCESS){
    //disable DMA
    DMA0CTL&=~DMAEN;
    DMA1CTL&=~DMAEN; 
    //SPI pins back to GPIO
    SPI_deactivate();
    //Return Error
    //TODO: better error code here
    return resp;
  }
  //calculate wait time based on packet length
  time=len/10;
  if(time<=BUS_SPI_MIN_TIMEOUT){
    time=BUS_SPI_MIN_TIMEOUT;
  }
  //wait for SPI complete signal from master
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&arcBus_stat.events,BUS_EV_SPI_MASTER,CTL_TIMEOUT_DELAY,time);
  //disable DMA
  DMA0CTL&=~DMAEN;
  DMA1CTL&=~DMAEN; 
  //SPI pins back to GPIO
  SPI_deactivate();
  //Check if SPI complete event received
  if(e&BUS_EV_SPI_COMPLETE){
    //if RX is null then don't calculate CRC
    if(rx!=NULL){
        //assemble CRC
        crc=((unsigned char*)rx)[arcBus_stat.spi_stat.len+1];//LSB
        crc|=(((unsigned short)((unsigned char*)rx)[arcBus_stat.spi_stat.len])<<8);//MSB
        //check CRC
        if(crc!=crc16(rx,arcBus_stat.spi_stat.len)){
          //Bad CRC
          return ERR_BAD_CRC;
        }
    }
    //Success!!
    return RET_SUCCESS;
  }else if(e&BUS_EV_SPI_NACK){
    char tmp=arcBus_stat.spi_stat.nack;
    //clear NACK reason
    arcBus_stat.spi_stat.nack=0;
    //check why NACK was sent
    switch(tmp){
      case ERR_PK_LEN:
        //not sure why this could have happened
        return ERR_INVALID_ARGUMENT;
      break;
      case ERR_SPI_LEN:
        //SPI data is bigger than the buffer
        return ERR_BAD_LEN;
      break;
      case ERR_SPI_BUSY:
      case ERR_BUFFER_BUSY:
        //the other MSP is busy
        return ERR_BUSY;
      break;
      default:
        return ERR_UNKNOWN;
    }
  }else{
    //Return error, timeout occurred
    return ERR_TIMEOUT;
  }
}

//assert one or more interrupts on the bus
void BUS_int_set(unsigned char set){
    //disable interrupts for the pins
    P1IE&=~set;
    //set output level to high
    P1OUT|=set;
    //set pins to output
    P1DIR|=set;
#ifdef CDH_LIB
    //if CDH set to full drive strength
    P1REN&=~set;
#endif        
}
    
//de-assert one or more interrupts on the bus
void BUS_int_clear(unsigned char clear){
#ifdef CDH_LIB
    //if CDH set to pull resistor
    P1REN|=clear;
#endif  
    //set pins to input
    P1DIR&=~clear;
#ifdef CDH_LIB
    //if CDH set to pull down
    P1OUT&=~clear;
#endif
    //clear interrupt flag
    P1IFG&=~clear;
    //enable interrupts for the pins
    P1IE|=clear;
}

//return which build is used
int BUS_build(void){
#ifdef CDH_LIB
    return BUS_BUILD_CDH;
#else
    return BUS_BUILD_SUBSYSTEM;
#endif
}
    
