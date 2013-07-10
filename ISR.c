#include <ctl.h>
#include <msp430.h>
#include <stdlib.h>
#include "timerA.h"
#include "ARCbus.h"
#include "spi.h"
#include "DMA.h"

#include "ARCbus_internal.h"

//buffer for ISR command receive
unsigned char i2c_buf[40];

//bus internal events
extern CTL_EVENT_SET_t BUS_INT_events;

//DMA events
CTL_EVENT_SET_t DMA_events;

//=======================================================================================
//                      [Interrupt Service Routines]
//=======================================================================================

void UC0_TX(void) __ctl_interrupt[USCIAB0TX_VECTOR]{
  unsigned char flags=UC0IFG&(UC0IE);
//==================[SPI TX Handler]==================
  //SPI communication is handled by DMA, no handler needed
//===============[I2C data Rx Handler]================
  if(flags&UCB0RXIFG){
    //receive data
    arcBus_stat.i2c_stat.rx.ptr[arcBus_stat.i2c_stat.rx.idx++]=UCB0RXBUF;
    //check mode
    if(!(UCB0CTL0&UCMST)){//slave mode
      //check buffer size
      if(arcBus_stat.i2c_stat.rx.idx>=sizeof(i2c_buf)){
        //receive buffer is full, send NACK
        UCB0CTL1|=UCTXNACK;
      }
    }else{//master mode
      /*//check if this is the 2nd to last byte
      if(arcBus_stat.i2c_stat.rx.len==(arcBus_stat.i2c_stat.rx.idx+1)){
          //generate stop condition
          UCB0CTL1|=UCTXSTP;
        //one more interrupt to go
      }
      //check if this was the last byte
      if(arcBus_stat.i2c_stat.rx.idx>=arcBus_stat.i2c_stat.rx.len){
        //signal that packet has been sent
        ctl_events_set_clear(&arcBus_stat.events,BUS_EV_I2C_COMPLETE,0);
        //set state to idle
        arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
        //disable I2C Rx Interrupts
        UC0IE&=~(UCB0RXIE);
      }*/
    }
  }
//===============[I2C data Tx Handler]================
  if(flags&UCB0TXIFG){
    //check if there are more bytes
    if(arcBus_stat.i2c_stat.tx.len>arcBus_stat.i2c_stat.tx.idx){
      //transmit data
      UCB0TXBUF=arcBus_stat.i2c_stat.tx.ptr[arcBus_stat.i2c_stat.tx.idx++];
    }else{//nothing left to send
      if(!(UCB0CTL0&UCMST)){//slave mode
        //no more data to send so send dummy data
        UCB0TXBUF=BUS_I2C_DUMMY_DATA;
      }else{//Master Mode
        //generate stop condition
        UCB0CTL1|=UCTXSTP;
        //if running in background a diffrent event must be set to release the mutex
        if(!arcBus_stat.i2c_stat.mutex_release){
          //set event
          ctl_events_set_clear(&arcBus_stat.events,BUS_EV_I2C_COMPLETE,0);
        }else{
          //set event
          ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_RELEASE_MUTEX,0);
        }
        //set state to idle
        arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
        //disable I2C Tx Interrupts
        UC0IE&=~(UCB0TXIE);
        //clear interrupt flag
        UC0IFG&= ~UCB0TXIFG;
      }
    }
  }

}


void UC0_rx(void) __ctl_interrupt[USCIAB0RX_VECTOR]{
  unsigned char flags=UC0IFG&(UC0IE);
  unsigned char len;
  int i;
//==================[SPI RX Handler]==================
  //SPI communication is handled by DMA, no handler needed
//=================[I2C Status Handler]=============================
  //NACK received
  if(UCB0STAT&UCNACKIFG){
    //Acknowledge expected but not received  
    //generate stop condition
    UCB0CTL1|=UCTXSTP; 
    //if running in background a diffrent event must be set to release the mutex
    if(!arcBus_stat.i2c_stat.mutex_release){
      //set ERROR flag
      ctl_events_set_clear(&arcBus_stat.events,BUS_EV_I2C_NACK,0);
    }else{
      //set event
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_RELEASE_MUTEX,0);
    }
    //set state to idle
    arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
    //clear interrupt flag
    UCB0STAT&=~UCNACKIFG;
    //disable I2C Tx and Rx Interrupts
    UC0IE&=~(UCB0TXIE|UCB0RXIE);
    //clear Tx interrupt flag see USCI25 in "MSP430F261x, MSP430F241x Device Erratasheet (Rev. J)"
    UC0IFG&= ~UCB0TXIFG;
  }
  //Stop condition received, end of command 
  if(UCB0STAT&UCSTPIFG){
    //check if transaction was a command
    if(arcBus_stat.i2c_stat.mode==BUS_I2C_RX){
      //set flag to notify 
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_I2C_CMD_RX,0);
    }
    //set state to idle
    arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
    //disable I2C Tx and Rx Interrupts
    UC0IE&=~(UCB0TXIE|UCB0RXIE);
    //disable stop interrupt
    UCB0I2CIE&=~UCSTPIE;
  }
  //start condition and slave address received, setup for command
  if(UCB0STAT&UCSTTIFG){
    //check status
    //This is to fix the issue where the start condition happens before the stop can be processed
    if(arcBus_stat.i2c_stat.mode==BUS_I2C_RX){
      //set flag to notify 
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_I2C_CMD_RX,0);
      //set state to idle
      arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
      //disable I2C Tx and Rx Interrupts
      UC0IE&=~(UCB0TXIE|UCB0RXIE);
    }
    //Check if transmitting or receiving
    if(UCB0CTL1&UCTR){          
      //enable I2C Tx Interrupt
      UC0IE|=UCB0TXIE;
      //zero index
      arcBus_stat.i2c_stat.tx.idx=0;
      //An I2C slave should always be the receiver
      arcBus_stat.i2c_stat.tx.ptr=NULL;
      arcBus_stat.i2c_stat.tx.len=-1;
      //set mode to Tx
      arcBus_stat.i2c_stat.mode=BUS_I2C_TX;
      //send first byte to save time
      UCB0TXBUF=BUS_I2C_DUMMY_DATA;
    }else{
      //enable I2C Rx Interrupt
      UC0IE|=UCB0RXIE;
      //setup receive status
      arcBus_stat.i2c_stat.rx.ptr=i2c_buf;
      arcBus_stat.i2c_stat.rx.len=sizeof(i2c_buf);
      arcBus_stat.i2c_stat.rx.idx=0;
      //set mode to Rx
      arcBus_stat.i2c_stat.mode=BUS_I2C_RX;
    }
    //enable stop interrupt
    UCB0I2CIE|=UCSTPIE;
    //clear start flag
    UCB0STAT&=~UCSTTIFG;
  }
  //arbitration lost, no longer master
  if(UCB0STAT&UCALIFG){
    //Arbitration lost, resend later?
    UCB0STAT&=~UCALIFG;
    //set status to idle
    arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
  }
}

//=================[Port pin Handler]=============================
void bus_int(void) __ctl_interrupt[PORT1_VECTOR]{
  unsigned char flags=P1IFG&P1IE;
  P1IFG&=~flags;
  //set events for flags
  ctl_events_set_clear(&arcBus_stat.PortEvents,flags,0);
  //TESTING: send time check event
  //ctl_events_set_clear(&SUB_events,SUB_EV_TIME_CHECK,0);
}


//================[DMA Transfer Complete]=========================
void DMA_int(void) __ctl_interrupt[DMA_VECTOR]{
  switch(DMAIV){
    case DMAIV_DMA0IFG:
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_SPI_COMPLETE,0);
    break;
    case DMAIV_DMA1IFG:
      ctl_events_set_clear(&DMA_events,DMA_EV_SD_SPI,0);
    break;
    case DMAIV_DMA2IFG:
      ctl_events_set_clear(&DMA_events,DMA_EV_USER,0);
    break;
  }
}

//================[Time Tick interrupt]=========================
void task_tick(void) __ctl_interrupt[TIMERA0_VECTOR]{
  extern ticker ticker_time;
  //set rate to 1024Hz
  TACCR0+=32;
  //update ticker time
  ticker_time++;
  //increment timer
  ctl_increment_tick_from_isr();

  if(async_timer){
    async_timer--;
    if(!async_timer){
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_ASYNC_TIMEOUT,0);
    }
  }
}
