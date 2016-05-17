#include <ctl.h>
#include <msp430.h>
#include <string.h>
#include <stdlib.h>
#include "timerA.h"
#include "ARCbus.h"
#include "crc.h"
#include "spi.h"

#include "ARCbus_internal.h"

__thread unsigned char BUS_thread_addr_flags=0;

//=======================================================================================
//                                 [BUS Functions]
//=======================================================================================

//return own address
unsigned char BUS_get_OA(void){
  int i;
  //base address for own I2C addresses
  volatile unsigned int * const oa_base=&UCB0I2COA0;
  unsigned char addr;
  //check for default address flags
  if(BUS_thread_addr_flags!=0){
    //try to get thread specific address
    addr=BUS_flags_to_addr(BUS_thread_addr_flags);
    //check for success
    if(!(addr&BUS_FLAGS_ADDR_MASK) && addr!=BUS_ADDR_GC){
      //return address
      return addr;
    }
    //report error
    report_error(ERR_LEV_ERROR,BUS_ERR_SRC_I2C,I2C_ERR_INVALID_FLAGS,(BUS_thread_addr_flags<<8)|(addr));
    //reset flags to default
    BUS_thread_addr_flags=0;
  }
  //Fallback loop through addresses and return the first one that is enabled
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
}//return own address


//set own address, this is done on a per-task basis
unsigned char BUS_set_OA(unsigned char addr){
  unsigned char flags;
  //get flags from address
  flags=BUS_addr_to_flags(addr);
  //check for error
  switch(flags){
    case BUS_FLAGS_ADDR_DISABLED:
    case BUS_FLAGS_INVALID_ADDR:
    case CMD_PARSE_GC_ADDR:
      //return error from BUS_addr_to_flags
      //TODO: is there a better way to do this?
      return flags;
    default:
      //set flags for this task
      BUS_thread_addr_flags=flags;
      //return success
      return RET_SUCCESS;
  }
}

//enable extra I2C own address registers
int BUS_I2C_aux_addr(unsigned char addr,unsigned char dest){
  switch(dest){
    case CMD_PARSE_ADDR1:
      //check if address is enabled
      if(UCOAEN&UCB0I2COA1){
        //error, address is already enabled
        return ERR_BAD_ADDR;
      }
      //enable interrupts for address 2
      UCB0IE|=UCTXIE1|UCRXIE1;
      //set and enable address 1
      UCB0I2COA1=UCOAEN|addr;
    break;
    case CMD_PARSE_ADDR2:
      //check if address is enabled
      if(UCOAEN&UCB0I2COA2){
        //error, address is already enabled
        return ERR_BAD_ADDR;
      }
      //enable interrupts for address 2
      UCB0IE|=UCTXIE2|UCRXIE2;
      //set and enable address 2
      UCB0I2COA2=UCOAEN|addr;
    break;
    case CMD_PARSE_ADDR3:
      //check if address is enabled
      if(UCOAEN&UCB0I2COA3){
        //error, address is already enabled
        return ERR_BAD_ADDR;
      }
      //enable interrupts for address 3
      UCB0IE|=UCTXIE3|UCRXIE3;
      //set and enable address 3
      UCB0I2COA3=UCOAEN|addr;
    break;
    default:
      //all other values are invalid
      return ERR_INVALID_ARGUMENT;
  }
  return RET_SUCCESS;
}

//return I2C address based on flags
unsigned char BUS_flags_to_addr(unsigned char flags){
  switch(flags){
    case CMD_PARSE_ADDR0:
      //check if address is enabled
      if(UCOAEN&UCB0I2COA0){
        //return address without GCEN or OAEN bits
        return (~(UCGCEN|UCOAEN))&UCB0I2COA0;
      }
    break;
    case CMD_PARSE_ADDR1:
      //check if address is enabled
      if(UCOAEN&UCB0I2COA1){
        //return address without GCEN or OAEN bits
        return (~(UCGCEN|UCOAEN))&UCB0I2COA1;
      }
    break;
    case CMD_PARSE_ADDR2:
      //check if address is enabled
      if(UCOAEN&UCB0I2COA2){
        //return address without GCEN or OAEN bits
        return (~(UCGCEN|UCOAEN))&UCB0I2COA2;
      }
    break;
    case CMD_PARSE_ADDR3:
      //check if address is enabled
      if(UCOAEN&UCB0I2COA3){
        //return address without GCEN or OAEN bits
        return (~(UCGCEN|UCOAEN))&UCB0I2COA3;
      }
    break;
    case CMD_PARSE_GC_ADDR:
      return BUS_ADDR_GC;
    default:
      return BUS_FLAGS_INVALID_ADDR;
  }
  return BUS_FLAGS_ADDR_DISABLED;
}

//find flags for address, address must be enabled
unsigned char BUS_addr_to_flags(unsigned char addr){
  int i,disabled;
  //base address for own I2C addresses
  volatile unsigned int * const oa_base=&UCB0I2COA0;
  //loop through addresses to find a match
  for(i=0,disabled;i<4;i++){
    //check for a match
    if(((~(UCGCEN|UCOAEN))&oa_base[i])==addr){
      //check if address is enabled
      if(oa_base[i]&UCOAEN){
        //Success!!, return flags for address
        return CMD_PARSE_ADDR0<<i;
      }else{
        //disabled, set flag
        disabled=1;
      }
    }
  }
  //TODO: are these the best error values to return??
  //check if address was disabled
  if(disabled){
    //return disabled error
    return BUS_FLAGS_ADDR_DISABLED;
  }else{
    //return invalid flags error
    return BUS_FLAGS_INVALID_ADDR;
  }
}//return own address

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
  if(0==ctl_mutex_lock(&arcBus_stat.i2c_stat.mutex,CTL_TIMEOUT_DELAY,100)){
     return ERR_BUSY;
  }
  return 0;
} 

//release the I2C bus
void BUS_I2C_release(void){
  ctl_mutex_unlock(&arcBus_stat.i2c_stat.mutex);
}

//keep track of which errors have happened
static int BUS_I2C_err_track(int error){
  //keep track of how many errors have happened
  static errors=0;
  //check which error happened
  switch(error){
    //I2C start timeout error happened
    case ERR_I2C_START_TIMEOUT:
      //This error causes problems reset after only a few errors
      if(errors>3){
        //reset MSP430 to clear the error
        reset(ERR_LEV_ERROR+20,BUS_ERR_SRC_I2C,I2C_ERR_TOO_MANY_ERRORS,error);
      }
      errors++;
    break;
    //These errors happen when the device is not found or busy
    case ERR_I2C_NACK:
    case BUS_EV_I2C_TX_SELF:
    case BUS_EV_I2C_ABORT:
      //Do nothing, errors are not cleared or incremented
    break;
    //send successful!
    case RET_SUCCESS:
      //reset error count
      errors=0;
    break;
    //Clock low timeout
    case BUS_EV_I2C_ERR_CCL:
      //This error does not happen too often
      if(errors>10){
        //reset MSP430 to clear the error
        reset(ERR_LEV_ERROR+20,BUS_ERR_SRC_I2C,I2C_ERR_TOO_MANY_ERRORS,error);
      }
      errors++;
    break;
    //Other or unknown error
    default:
      //reset if a lot of these happen
      if(errors>40){
        //reset MSP430 to clear the error
        reset(ERR_LEV_ERROR+20,BUS_ERR_SRC_I2C,I2C_ERR_TOO_MANY_ERRORS,error);
      }
      errors++;
    break;
  }
  //release I2C bus
  BUS_I2C_release();
  //return error
  return error;
}

//send command
int BUS_cmd_tx(unsigned char addr,void *buff,unsigned short len,unsigned short flags){
  unsigned int e;
  short ret;
  int i;
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
  //check if a software transmission should be done
  if(!(flags&BUS_CMD_FL_NO_SW_TX) && (BUS_OA_check(addr)==ERR_BAD_ADDR || addr==BUS_ADDR_GC)){
    for(i=0;;i++){
      //disable interrupts
      int en=ctl_global_interrupts_disable();
      //check if buffer is in use
      if(I2C_rx_buf[I2C_rx_in].stat==I2C_PACKET_STAT_EMPTY){
        //check if sending to the general call address
        if(addr==BUS_ADDR_GC){
          //set flags for general call address
          I2C_rx_buf[I2C_rx_in].flags=BUS_FLAGS_SW_GC|BUS_thread_addr_flags;
        }else{
          //set flags
          I2C_rx_buf[I2C_rx_in].flags=BUS_addr_to_flags(addr);
        }
        //copy data into buffer
        memcpy(I2C_rx_buf[I2C_rx_in].dat,buff,len);
        //set length
        I2C_rx_buf[I2C_rx_in].len=len;
        //set buffer status
        I2C_rx_buf[I2C_rx_in].stat=I2C_PACKET_STAT_COMPLETE;
        //increment index
        I2C_rx_in++;
        //check for wraparound
        if(I2C_rx_in>=BUS_I2C_PACKET_QUEUE_LEN){
          I2C_rx_in=0;
        }
        //enable interrupts
        if(en){
          ctl_global_interrupts_enable();
        }
        //set flag for new packet
        ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_I2C_CMD_RX,0);
        //return here because transaction is complete
        return RET_SUCCESS;
      }else{
        //re-enable interrupts
        if(en){
          ctl_global_interrupts_enable();
        }
        //check number of retries
        if(i>4){
          //too many retries, return error
          return ERR_BUSY;
        }else{
          //wait for packet reception to finish
          ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&arcBus_stat.events,BUS_EV_I2C_RX_DONE,CTL_TIMEOUT_DELAY,50);
        }
      }
    }
  }
  //wait for the bus to become free
  if(BUS_I2C_lock()){
    //I2C bus is in use
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
  //wait for packet to start
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&arcBus_stat.events,BUS_EV_I2C_MASTER_START,CTL_TIMEOUT_DELAY,50);
  //check to see if there was a problem
  if(!(e&BUS_EV_I2C_MASTER_STARTED)){
    //clear start bit
    UCB0CTL1&=~UCTXSTT;
    //set I2C master state
    arcBus_stat.i2c_stat.tx.stat=BUS_I2C_MASTER_IDLE;
    //chech which error happened
    switch(e&BUS_EV_I2C_MASTER_START){
      case 0:
        //no event happened so timeout
        return BUS_I2C_err_track(ERR_I2C_START_TIMEOUT);
      case BUS_EV_I2C_NACK:
        //I2C device did not acknowledge
        return BUS_I2C_err_track(ERR_I2C_NACK);
      default:
        //error is not defined
        return BUS_I2C_err_track(ERR_UNKNOWN);
    }
  }
  //wait for transaction to complete
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&arcBus_stat.events,BUS_EV_I2C_MASTER,CTL_TIMEOUT_DELAY,50);
  //save transaction time
  packet_time=get_ticker_time();
  //set I2C master state
  arcBus_stat.i2c_stat.tx.stat=BUS_I2C_MASTER_IDLE;
  //check which event(s) happened
  switch(e&BUS_EV_I2C_MASTER){
    case BUS_EV_I2C_COMPLETE:
      //no error
      return BUS_I2C_err_track(RET_SUCCESS);
    case BUS_EV_I2C_NACK:
      //I2C device did not acknowledge
      return BUS_I2C_err_track(ERR_I2C_NACK);
    case BUS_EV_I2C_ABORT:
      //I2C device did not acknowledge
      return BUS_I2C_err_track(ERR_I2C_ABORT);
    case 0:
      //no event happened, so time out
      return BUS_I2C_err_track(ERR_TIMEOUT);
    case BUS_EV_I2C_ERR_CCL:
      //Clock low timeout
      return BUS_I2C_err_track(ERR_I2C_CLL);
    case BUS_EV_I2C_TX_SELF:
      //TX to self and no one else responded
      return BUS_I2C_err_track(ERR_I2C_TX_SELF);
    default:
      //error is not defined
      return BUS_I2C_err_track(ERR_UNKNOWN);
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
  DMA2CTL&=~DMAEN;
  //Setup SPI
  SPI_slave_setup();
  //setup DMA for transfer
  DMACTL0 &=~(DMA0TSEL_31|DMA1TSEL_31);
  DMACTL0 |= (DMA0TSEL__USCIA0RX|DMA1TSEL__USCIA0TX);
  DMACTL1 = DMA2TSEL__USCIA0TX;
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
    DMA0CTL =DMADT_0|DMASBDB|DMAEN|DMASRCINCR_3|DMADSTINCR_0;
  }
  //====[DMA channel2 used for DMA9 fix]====
  //DMA9 workaround, use a dummy channel with lower priority and the same trigger
  //setup dummy channel: read and write from unused space in the SPI registers
  *((unsigned int*)&DMA2SA) = EUSCI_A0_BASE + 0x02;
  *((unsigned int*)&DMA2DA) = EUSCI_A0_BASE + 0x04;
  // only one byte
  DMA2SZ = 1;
  // Configure the DMA transfer, repeated byte transfer with no increment
  DMA2CTL = DMADT_4|DMASBDB|DMAEN|DMASRCINCR_0|DMADSTINCR_0;
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
    DMA1CTL=DMADT_0|DMASBDB|DMASRCINCR_3|DMADSTINCR_0|DMAEN;
    //start things off with an initial transfer
    UCA0TXBUF=*((unsigned char*)tx);
  }else{
    //need to send something to receive something so setup TX for dummy bytes
    *((unsigned int*)&DMA1SA) = (unsigned int)(&UCA0TXBUF);
    // The size of the block to be transferred
    DMA1SZ = len+BUS_SPI_CRC_LEN-1;
    // Configure the DMA transfer, single byte transfer with no increment
    DMA1CTL=DMADT_0|DMASBDB|DMASRCINCR_0|DMADSTINCR_0|DMAEN;
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
  resp=BUS_cmd_tx(addr,buf,2,BUS_CMD_FL_NACK);
  //check if sent correctly
  if(resp!=RET_SUCCESS){
    //disable DMA
    DMA0CTL&=~DMAEN;
    DMA1CTL&=~DMAEN; 
    DMA2CTL&=~DMAEN; 
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
  DMA2CTL&=~DMAEN; 
  //SPI pins back to GPIO
  SPI_deactivate();
  //Check if SPI complete event received
  if(e&BUS_EV_SPI_COMPLETE){
    //check for errors from the destination
    if(arcBus_stat.spi_stat.nack!=0){
      //error from the other system, return it
      return arcBus_stat.spi_stat.nack;
    }
    //if RX is null then don't calculate CRC
    if(rx!=NULL){
        //check if DMA0 finished receiving 
        if(!(DMA0CTL&DMAIFG)){
          //Error : DMA timed out (CRC is probably bad)
          return ERR_DMA_TIMEOUT;
        }
        //assemble CRC
        crc=((unsigned char*)rx)[arcBus_stat.spi_stat.len+1];//LSB
        crc|=(((unsigned short)((unsigned char*)rx)[arcBus_stat.spi_stat.len])<<8);//MSB
        //check CRC
        if(crc!=crc16(rx,arcBus_stat.spi_stat.len)){
          //Bad CRC
          return ERR_BAD_CRC;
        }
    }
    //check if DMA1 finished transmitting
    if(!(DMA1CTL&DMAIFG)){
      //Error : DMA timed out (CRC is probably bad on the other end)
      return ERR_DMA_TIMEOUT;
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
    //timeout occurred, send SPI abort packet
    ptr=BUS_cmd_init(buf,CMD_SPI_ABORT);
    resp=BUS_cmd_tx(addr,buf,0,BUS_CMD_FL_NACK);
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

//timeout delay for time specified in milliseconds    
void BUS_delay_msec(CTL_TIME_t timeout){
  //TODO: calculate time in msec
  //make sure timeout is greater than two
  if(timeout<2){
    //set timeout to minimum
    timeout=2;
  }
  //wait for time to expire
  ctl_timeout_wait(ctl_get_current_time()+timeout);
}

//timeout delay for time specified in microseconds
void BUS_delay_usec(CTL_TIME_t timeout){
  //calculate time in usec
  timeout=(timeout+488)/977;
  //make sure timeout is greater than two
  if(timeout<2){
    //set timeout to minimum
    timeout=2;
  }
  //wait for time to expire
  ctl_timeout_wait(ctl_get_current_time()+timeout);
}
 
