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

//send command
int BUS_cmd_tx(unsigned char addr,unsigned char *buff,unsigned short len,unsigned short flags,short bgnd){
  unsigned int e;
  short ret;
  int i;
  unsigned char resp[2];
  //add standard header length
  len+=I2C_HDR_LEN;
  //add NACK flag if requested
  if(flags&BUS_CMD_FL_NACK){
    //request NACK
    buff[0]|=CMD_TX_NACK;
  }else{
    //clear NACK request
    buff[0]&=~CMD_TX_NACK;
  }
  //calculate CRC
  buff[len]=crc8(buff,len);
  //add a byte for the CRC
  len+=I2C_CRC_LEN;
  //send command without ack
  ret=BUS_i2c_tx(addr,buff,len);
  //check for error
  if(ret!=RET_SUCCESS){
    //return error
    return ret;
  }
  //if transmitting in the background, return
  if(bgnd){
    return RET_SUCCESS;
  }
  //wait for transaction to complete
  //TODO: set a good timeout
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&arcBus_stat.events,BUS_EV_I2C_MASTER,CTL_TIMEOUT_DELAY,2048);
  //check event for error
  if(e==BUS_EV_I2C_COMPLETE){
      //no error
      return RET_SUCCESS;
  }else{
    //return error
    //TODO: give more informative error messages
    return ERR_UNKNOWN;
  }
}

int BUS_SPI_txrx(unsigned char addr,unsigned char *tx,unsigned char *rx,unsigned short len){
  unsigned char buf[10],*ptr;
  unsigned int e;
  short time;
  int i,resp;
  unsigned short crc;
  //reject General call address
  if(addr==BUS_ADDR_GC){
    return ERR_BADD_ADDR;
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
    DMA0SZ = len+SPI_CRC_LEN;
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
    DMA1SZ = len+SPI_CRC_LEN-1;
    // Configure the DMA transfer, single byte transfer with destination increment
    //enable interrupt to notify code when transfer is complete
    DMA1CTL=DMADT_0|DMASBDB|DMASRCINCR1|DMASRCINCR0|DMAEN;
    //start things off with an initial transfer
    UCA0TXBUF=*tx;
  }else{
    //need to send something to receive something so setup TX for dummy bytes
    DMA1SA = (unsigned int)(&UCA0TXBUF);
    // The size of the block to be transferred
    DMA1SZ = len+SPI_CRC_LEN-1;
    // Configure the DMA transfer, single byte transfer with no increment
    DMA1CTL=DMADT_0|DMASBDB|DMASRCINCR0|DMASRCINCR0|DMAEN;
    //start things off with an initial transfer
    UCA0TXBUF=SPI_DUMMY_DATA;
  }
  //send SPI setup command
  ptr=BUS_cmd_init(buf,CMD_SPI_RDY);
  //send MSB first
  ptr[0]=len>>8;
  //then send LSB
  ptr[1]=len;
  //send command
  resp=BUS_cmd_tx(addr,buf,2,BUS_CMD_FL_NACK,SEND_FOREGROUND);
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
    //assemble CRC
    crc=rx[arcBus_stat.spi_stat.len+1];//LSB
    crc|=(((unsigned short)rx[arcBus_stat.spi_stat.len])<<8);//MSB
    //check CRC
    if(crc!=crc16(rx,arcBus_stat.spi_stat.len)){
      //Bad CRC
      return ERR_BAD_CRC;
    }
    //Success!!
    return RET_SUCCESS;
  }else{
    //Return error, timeout occurred
    return ERR_TIMEOUT;
  }
}

//=======================================================================================
//                                 [I2C Functions]
//=======================================================================================

//become master on the I2C bus and transmit len bytes pointed to by dat to address addr
short BUS_i2c_tx(unsigned short addr,const unsigned char *dat,unsigned short len){
  unsigned int e;
  //check for zero length
  if(len==0){
    return ERR_BAD_LEN;
  }
  //wait for until not transmitting or receiving
  while(arcBus_stat.i2c_stat.mode!=I2C_IDLE){
    ctl_timeout_wait(ctl_get_current_time()+3);
  }
  //wait for the bus to become free
  /*while(UCB0STAT&UCBBUSY || arcBus_stat.i2c_stat.mode!=I2C_IDLE){
    ctl_timeout_wait(ctl_get_current_time()+3);
  }
  ctl_timeout_wait(ctl_get_current_time()+3);*/
    
  //Setup for I2C transaction  
  //set slave address
  UCB0I2CSA=addr;
  //set index
  arcBus_stat.i2c_stat.tx.idx=0;
  //set mode
  arcBus_stat.i2c_stat.mode=I2C_TX;
  //set length
  arcBus_stat.i2c_stat.tx.len=len;
  //set data
  arcBus_stat.i2c_stat.tx.ptr=(unsigned char*)dat;
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
  for(;;){
    if(!(UCB0STAT&UCBBUSY)){
      ctl_timeout_wait(ctl_get_current_time()+3);
      if(!(UCB0STAT&UCBBUSY)){
        break;
      }
    }
    ctl_timeout_wait(ctl_get_current_time()+3);
  }
  //enable I2C Tx Interrupt
  UC0IE|=UCB0TXIE;
  //generate start condition
  UCB0CTL1|=UCTXSTT;
  //sending started return success
  return RET_SUCCESS;
}

//become master on the I2C bus and transmit txLen bytes then recive rxlen bytes
/*short BUS_i2c_txrx(unsigned short addr,const unsigned char *tx,unsigned short txLen,unsigned char *rx,unsigned short rxLen){
  unsigned int e;
  //check for incorrect length
  if(txLen==0 ||rxLen<=1){
    return ERR_BAD_LEN;
  }
  //wait for the bus to become free
  while(UCB0STAT&UCBBUSY || arcBus_stat.i2c_stat.mode!=I2C_IDLE){
    ctl_timeout_wait(ctl_get_current_time()+3);
  }
  //set slave address
  UCB0I2CSA=addr;
  //set index
  arcBus_stat.i2c_stat.tx.idx=0;
  //set mode
  arcBus_stat.i2c_stat.mode=I2C_TXRX;
  //set length
  arcBus_stat.i2c_stat.tx.len=txLen;
  //set data
  arcBus_stat.i2c_stat.tx.ptr=(unsigned char*)tx;
  //set rx index
  arcBus_stat.i2c_stat.rx.idx=0;
  //set rx length
  arcBus_stat.i2c_stat.rx.len=rxLen;
  //set data
  arcBus_stat.i2c_stat.rx.ptr=(unsigned char*)rx;
  //check if master and switch if necessary
  if(!(UCB0CTL0&UCMST)){
    //reset peripheral to switch to master mode
    UCB0CTL1|=UCSWRST;
    //set master mode
    UCB0CTL0|=UCMST;
    //take peripheral out of reset
    UCB0CTL1&=~UCSWRST;
  }
  //set transmit mode
  UCB0CTL1|=UCTR;
  //clear master I2C flags
  ctl_events_set_clear(&arcBus_stat.events,0,BUS_EV_I2C_MASTER);
  //enable I2C Tx Interrupt
  UC0IE|=UCB0TXIE;
  //generate start condition
  UCB0CTL1|=UCTXSTT;
  //sending started return success
  return RET_SUCCESS;
}*/