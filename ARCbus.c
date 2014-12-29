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

//Setup buffer for command 
unsigned char *BUS_cmd_init(unsigned char *buf,unsigned char id){
  buf[1]=id;
  //set originator address
  buf[0]=UCB0I2COA;
  //start of payload
  return buf+2;
}

//function to check I2C addresses
int addr_chk(unsigned char addr){
  if(addr==UCB0I2COA){
    return ERR_BAD_ADDR;
  }
  if(addr&0x80){
    return ERR_BAD_ADDR;
  }
  return RET_SUCCESS;
}


static unsigned BUS_I2C_lock(void){
    int i;
    ticker tt;
    unsigned long delay;
    unsigned char addr;
    unsigned short slt,st;
    const unsigned char addr_slot[BUS_NUM_SLOTS]={BUS_ADDR_CDH,INVALID_I2C_ADDR,INVALID_I2C_ADDR,BUS_ADDR_LEDL,BUS_ADDR_ACDS,BUS_ADDR_COMM,BUS_ADDR_IMG,BUS_ADDR_LEDL};    
    //get address
    addr=UCB0I2COA&0x7F;
    #ifndef CDH_LIB
        //check that time has been updated
        if(!timesync){
            return ERR_TIME_INVALID;
        }
        //check that time was synced within the last minuet
        if((get_ticker_time()-timesync)>(1024*60)){
            return ERR_TIME_TOO_OLD;
        }
    #endif
    //try to capture mutex
    if(0==ctl_mutex_lock(&arcBus_stat.i2c_stat.mutex,CTL_TIMEOUT_DELAY,BUS_NUM_SLOTS*BUS_SLOT_TIME_LEN*2)){
        return ERR_BUSY;
    }
    for(i=0;i<2*BUS_NUM_SLOTS+2;i++){
        //check timeslot
        tt=get_ticker_time();
        //calculate time slot
        slt=(tt>>BUS_SLOT_NUM_SHIFT)&BUS_SLOT_NUM_MASK;
        //calculate slot time
        st=tt&BUS_SLOT_TIME_MASK;
        //check time slot and remaining time
        if(addr_slot[slt]==addr && st<(BUS_SLOT_TIME_LEN-BUS_MAX_PACKET_TIME)){
            return RET_SUCCESS;
        }
        //calculate delay to next slot
        delay=BUS_SLOT_TIME_LEN-st;
        //ensure minimum delay
        if(delay<3){
            delay=3;
        }
        //wait for next slot
        ctl_timeout_wait(ctl_get_current_time()+delay);
    }
    //release mutex
    BUS_I2C_release();
    //return error
    return ERR_BUSY;
} 

//release the I2C bus
void BUS_I2C_release(void){
  ctl_mutex_unlock(&arcBus_stat.i2c_stat.mutex);
}

//send command
int BUS_cmd_tx(unsigned char addr,unsigned char *buff,unsigned short len,unsigned short flags,short bgnd){
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
    buff[0]|=CMD_TX_NACK;
  }else{
    //clear NACK request
    buff[0]&=~CMD_TX_NACK;
  }
  //calculate CRC
  buff[len]=crc7(buff,len);
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
  //set mode
  arcBus_stat.i2c_stat.mode=BUS_I2C_TX;
  //set length
  arcBus_stat.i2c_stat.tx.len=len;
  //set data
  arcBus_stat.i2c_stat.tx.ptr=(unsigned char*)buff;
  //reset peripheral to switch to master mode
  UCB0CTL1|=UCSWRST;
  //set master mode
  UCB0CTL0|=UCMST;
  //set transmit mode
  UCB0CTL1|=UCTR;
  //take peripheral out of reset
  UCB0CTL1&=~UCSWRST;
  //clear master I2C flags
  ctl_events_set_clear(&arcBus_stat.events,0,BUS_EV_I2C_MASTER);
  //enable I2C Tx Interrupt
  UC0IE|=UCB0TXIE;
  //generate start condition
  UCB0CTL1|=UCTXSTT;
  //if transmitting in the background, return
  if(bgnd){
    //return success
    return RET_SUCCESS;
  }
  //wait for transaction to complete
  //TODO: set a good timeout
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&arcBus_stat.events,BUS_EV_I2C_MASTER,CTL_TIMEOUT_DELAY,2048);
  //release I2C bus
  BUS_I2C_release();
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
    default:
      //error is not defined
      return ERR_UNKNOWN;
  }
}

//send/receive SPI data over the bus
int BUS_SPI_txrx(unsigned char addr,unsigned char *tx,unsigned char *rx,unsigned short len){
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
  //reject General call address
  if(addr==BUS_ADDR_GC){
    return ERR_BAD_ADDR;
  }
  //calculate CRC
  crc=crc16(tx,len);
  //send CRC in Big endian order
  tx[len]=crc>>8;
  tx[len+1]=crc;
  //setup SPI structure
  arcBus_stat.spi_stat.len=len;
  arcBus_stat.spi_stat.rx=rx;
  arcBus_stat.spi_stat.tx=tx;
  //Setup SPI
  SPI_slave_setup();
  //disable DMA
  DMA0CTL&=~DMAEN;
  DMA1CTL&=~DMAEN;
  //setup DMA for transfer
  DMACTL0 &=~(DMA0TSEL_15|DMA1TSEL_15);
  DMACTL0 |= (DMA0TSEL_3|DMA1TSEL_4);
  //====[DMA channel0 used for receive]====
  //check for omitted receive buffer
  if(rx!=NULL){
    // Source DMA address: receive register.
    DMA0SA = (unsigned short)(&UCA0RXBUF);
    // Destination DMA address: rx buffer.
    DMA0DA = (unsigned short)rx;
    // The size of the block to be transferred
    DMA0SZ = len+BUS_SPI_CRC_LEN;
    // Configure the DMA transfer, single byte transfer with source increment
    DMA0CTL =DMADT_0|DMASBDB|DMAEN|DMADSTINCR1|DMADSTINCR0;
  }
  //====[DMA channel1 used for transmit]====
  // Destination DMA address: the transmit buffer.
  DMA1DA = (unsigned int)(&UCA0TXBUF);
  //check for omitted transmit buffer
  if(tx!=NULL){
    // Source DMA address: tx buffer
    DMA1SA = (unsigned int)tx+1;
    // The size of the block to be transferred
    DMA1SZ = len+BUS_SPI_CRC_LEN-1;
    // Configure the DMA transfer, single byte transfer with destination increment
    //enable interrupt to notify code when transfer is complete
    DMA1CTL=DMADT_0|DMASBDB|DMASRCINCR1|DMASRCINCR0|DMAEN;
    //start things off with an initial transfer
    UCA0TXBUF=*tx;
  }else{
    //need to send something to receive something so setup TX for dummy bytes
    DMA1SA = (unsigned int)(&UCA0TXBUF);
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
    //SPI pins back to GPIO
    SPI_deactivate();
    //Return Error
    //TODO: better error code here
    return resp;
  }
  //calculate wait time based on packet length
  time=len/10;
  if(time<=10){
    time=10;
  }
  //wait for SPI complete signal from master
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&arcBus_stat.events,BUS_EV_SPI_MASTER,CTL_TIMEOUT_DELAY,time);
  //disable DMA
  DMA0CTL&=~DMAEN;
  DMA1CTL&=~DMAEN; 
  //Check if SPI complete event received
  if(e&BUS_EV_SPI_COMPLETE){
    //if RX is null then don't calculate CRC
    if(rx!=NULL){
        //assemble CRC
        crc=rx[arcBus_stat.spi_stat.len+1];//LSB
        crc|=(((unsigned short)rx[arcBus_stat.spi_stat.len])<<8);//MSB
        //check CRC
        if(crc!=crc16(rx,arcBus_stat.spi_stat.len)){
          //Bad CRC
          return ERR_BAD_CRC;
        }
    }
    //Success!!
    return RET_SUCCESS;
  }else if(e&BUS_EV_SPI_NACK){
    return ERR_BUSY;
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

