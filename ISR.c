#include <ctl.h>
#include <msp430.h>
#include <stdlib.h>
#include "timerA.h"
#include "ARCbus.h"
#include "spi.h"
#include "DMA.h"

#include "ARCbus_internal.h"

//buffer for ISR command receive
I2C_PACKET I2C_rx_buf[BUS_I2C_PACKET_QUEUE_LEN];
//queue indexes
short I2C_rx_in,I2C_rx_out;

//DMA events
CTL_EVENT_SET_t DMA_events;

//=======================================================================================
//                      [Interrupt Service Routines]
//=======================================================================================

void bus_I2C_isr(void) __ctl_interrupt[USCI_B0_VECTOR]{
  static unsigned short end_e=0;
  switch(UCB0IV){
    case USCI_I2C_UCALIFG:    //Arbitration lost
      //Arbitration lost, resend later?
      //set flag to indicate condition
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_I2C_ARB_LOST,0);
      //check if running
      if(arcBus_stat.i2c_stat.mode!=BUS_I2C_IDLE){
        //set status to idle
        arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
      }
      //reset rx buffer status if in progress
      if(I2C_rx_buf[I2C_rx_in].stat==I2C_PACKET_STAT_IN_PROGRESS){
        //reset packet flags
        I2C_rx_buf[I2C_rx_in].stat=I2C_PACKET_STAT_EMPTY;
        //decrement packet index
      }
    break;
    case USCI_I2C_UCNACKIFG:    //NACK interrupt  
      //Acknowledge expected but not received  
      //generate stop condition
      UCB0CTL1|=UCTXSTP; 
      //check if we have written more than a byte to the TX buffer
      //one byte is always written after the start condition is sent
      if(arcBus_stat.i2c_stat.tx.idx>1){
          //check if end_e is set
          if(end_e==0){
            //set ABORT as the end event
            end_e=BUS_EV_I2C_ABORT;
          }
      }else{
          //set NACK as end event
          end_e=BUS_EV_I2C_NACK;
      }
    break;
    case USCI_I2C_UCSTTIFG:    //start condition received
      //check if we are master
      if(UCB0CTLW0&UCMST){
        //clear NACK IFG
        UCB0IFG&=~UCNACKIFG;
        //no processing necessary
        break;
      }
      //check status
      //This is to fix the issue where the start condition happens before the stop can be processed
      if(arcBus_stat.i2c_stat.mode==BUS_I2C_RX){
        //set flag to notify 
        ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_I2C_CMD_RX,0);
        //set state to idle
        arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
      }
      //Check if transmitting or receiving
      if(UCB0CTL1&UCTR){   
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
        //check buffer status
        if(I2C_rx_buf[I2C_rx_in].stat!=I2C_PACKET_STAT_EMPTY){
          //buffer is in-use transmit NACK
          UCB0CTL1|=UCTXNACK;
          //set flag to indicate an error
          ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_I2C_RX_BUSY,0);
        }else{
          //setup receive status
          arcBus_stat.i2c_stat.rx.ptr=I2C_rx_buf[I2C_rx_in].dat;
          arcBus_stat.i2c_stat.rx.len=sizeof(I2C_rx_buf[0].dat);
          arcBus_stat.i2c_stat.rx.idx=0;
          //set mode to Rx
          arcBus_stat.i2c_stat.mode=BUS_I2C_RX;
          //set buffer status
          I2C_rx_buf[I2C_rx_in].stat=I2C_PACKET_STAT_IN_PROGRESS;
        }
      }
    break;
    case USCI_I2C_UCSTPIFG:    //Stop condition received
      //check if we are master
      if(UCB0CTLW0&UCMST){
        //check if mutex release event should be sent
        if(!arcBus_stat.i2c_stat.mutex_release){
          //set saved event and clear TX self event
          ctl_events_set_clear(&arcBus_stat.events,end_e,0);
        }else{
          //set event
          ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_RELEASE_MUTEX,0);
          //clear flag
          arcBus_stat.i2c_stat.mutex_release=0;
        }
        //clear saved event
        end_e=0;
        //set state to idle
        arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
      }else{
        //clear start condition interrupt
        UCB0IFG&=~UCSTTIFG;
        //check if transaction was a command
        if(arcBus_stat.i2c_stat.mode==BUS_I2C_RX){
          //set packet length
          I2C_rx_buf[I2C_rx_in].len=arcBus_stat.i2c_stat.rx.idx;
          //zero rx index
          arcBus_stat.i2c_stat.rx.idx=0;
          //set buffer status to complete
          I2C_rx_buf[I2C_rx_in].stat=I2C_PACKET_STAT_COMPLETE;
          //increment index
          I2C_rx_in++;
          //check for wraparound
          if(I2C_rx_in>=BUS_I2C_PACKET_QUEUE_LEN){
            I2C_rx_in=0;
          }
          //set flag to notify 
          ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_I2C_CMD_RX,0);
        }
        //set state to idle
        arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
        //check master status to see if a command is pending
        if(arcBus_stat.i2c_stat.tx.stat==BUS_I2C_MASTER_PENDING){          
          //transmision interrupted, start again
          //set to transmit mode
          UCB0CTLW0|=UCTR;
          //clear master I2C flags
          ctl_events_set_clear(&arcBus_stat.events,0,BUS_EV_I2C_MASTER|BUS_EV_I2C_MASTER_START);
          //set master mode
          UCB0CTLW0|=UCMST;
          //clear saved event
          end_e=0; 
          //generate start condition
          UCB0CTL1|=UCTXSTT;
        }
      }
    break;
    case USCI_I2C_UCRXIFG3:    //Slave 3 RXIFG
      //check buffer size
      if(arcBus_stat.i2c_stat.rx.idx>=sizeof(I2C_rx_buf[0].dat)){
        //receive buffer is full, send NACK
        UCB0CTL1|=UCTXNACK;
      }else{
        //receive data
        arcBus_stat.i2c_stat.rx.ptr[arcBus_stat.i2c_stat.rx.idx++]=UCB0RXBUF;
      }
    break;
    case USCI_I2C_UCTXIFG3:    //Slave 3 TXIFG
        //no data to send so send dummy data
        UCB0TXBUF=BUS_I2C_DUMMY_DATA;
    break;
    break;
    case USCI_I2C_UCRXIFG2:    //Slave 2 RXIFG
      //check buffer size
      if(arcBus_stat.i2c_stat.rx.idx>=sizeof(I2C_rx_buf[0].dat)){
        //receive buffer is full, send NACK
        UCB0CTL1|=UCTXNACK;
      }else{
        //receive data
        arcBus_stat.i2c_stat.rx.ptr[arcBus_stat.i2c_stat.rx.idx++]=UCB0RXBUF;
      }
    break;
    case USCI_I2C_UCTXIFG2:    //Slave 2 TXIFG
    break;
    break;
    case USCI_I2C_UCRXIFG1:    //Slave 1 RXIFG
    break;
    case USCI_I2C_UCTXIFG1:    //Slave 1 TXIFG
    break;
    case USCI_I2C_UCRXIFG0:    //Data receive in master mode and Slave 0 RXIFG
      //check if master transaction is in progress
      if(arcBus_stat.i2c_stat.tx.stat==BUS_I2C_MASTER_IN_PROGRESS){
        //master transaction, nack as a slave
        UCB0CTL1|=UCTXNACK;
        //set send to self event
        end_e=BUS_INT_EV_I2C_TX_SELF;
        break;
      }  
      //check buffer size
      if(arcBus_stat.i2c_stat.rx.idx>=sizeof(I2C_rx_buf[0].dat)){
        //receive buffer is full, send NACK
        UCB0CTL1|=UCTXNACK;
      }else{
        //receive data
        arcBus_stat.i2c_stat.rx.ptr[arcBus_stat.i2c_stat.rx.idx++]=UCB0RXBUF;
      }
    break;
    case USCI_I2C_UCTXIFG0:    //Data transmit in master mode and Slave 0 TXIFG
      //check master status 
      if(arcBus_stat.i2c_stat.tx.stat==BUS_I2C_MASTER_PENDING){
        //set I2C master state
        arcBus_stat.i2c_stat.tx.stat=BUS_I2C_MASTER_IN_PROGRESS;
        //set state to tx
        arcBus_stat.i2c_stat.mode=BUS_I2C_TX;
        //set flag to notify 
        ctl_events_set_clear(&arcBus_stat.events,BUS_EV_I2C_MASTER_STARTED,0);
      }
      //check if there are more bytes
      if(arcBus_stat.i2c_stat.tx.len>arcBus_stat.i2c_stat.tx.idx){
        //transmit data
        UCB0TXBUF=arcBus_stat.i2c_stat.tx.ptr[arcBus_stat.i2c_stat.tx.idx++];
      }else{//nothing left to send
        if(!(UCB0CTLW0&UCMST)){//slave mode
          //no more data to send so send dummy data
          UCB0TXBUF=BUS_I2C_DUMMY_DATA;
        }else{//Master Mode
          //generate stop condition
          UCB0CTL1|=UCTXSTP;
          //if running in background a diffrent event must be set to release the mutex
          if(!arcBus_stat.i2c_stat.mutex_release){
            //set end event
            end_e=BUS_EV_I2C_COMPLETE;
          }
          //set state to idle
          arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
        }
      }
    break;
    case USCI_I2C_UCBCNTIFG:    //Byte Counter Zero
    break;
    case USCI_I2C_UCCLTOIFG:    //Cock low timeout
      //check if master or slave
      if(UCB0CTLW0&UCMST){
        //master mode, generate stop condition
        UCB0CTL1|=UCTXSTP;
        //set end event
        end_e=ERR_I2C_CLL;
      }else{
        //slave mode, send NACK
        UCB0CTL1|=UCTXNACK;
      }
      //set event
      ctl_events_set_clear(&arcBus_stat.events,BUS_EV_I2C_ERR_CCL,0);
    break;
    case USCI_I2C_UCBIT9IFG:    //9th bit interrupt
    break;
  }
}



//=================[SPI handler]=============================
void bus_SPI_isr(void) __ctl_interrupt[USCI_A0_VECTOR]{
  int tmp;
  //DUMMY ISR, not used
  tmp=UCA0IV;
}

//=================[Port pin Handler]=============================
void bus_int(void) __ctl_interrupt[PORT2_VECTOR]{
  switch(P2IV){
    case P1IV_P1IFG0:
      //set events for flags
      ctl_events_set_clear(&SUB_events,SUB_EV_INT_0,0);
    return;
    case P1IV_P1IFG1:
      //set events for flags
      ctl_events_set_clear(&SUB_events,SUB_EV_INT_1,0);
    return;
    case P1IV_P1IFG2:
      //set events for flags
      ctl_events_set_clear(&SUB_events,SUB_EV_INT_2,0);
    return;
    case P1IV_P1IFG3:
      //set events for flags
      ctl_events_set_clear(&SUB_events,SUB_EV_INT_3,0);
    return;
    case P1IV_P1IFG4:
      //set events for flags
      ctl_events_set_clear(&SUB_events,SUB_EV_INT_4,0);
    return;
    case P1IV_P1IFG5:
      //set events for flags
      ctl_events_set_clear(&SUB_events,SUB_EV_INT_5,0);
    return;
    case P1IV_P1IFG6:
      //set events for flags
      ctl_events_set_clear(&SUB_events,SUB_EV_INT_6,0);
    return;
    case P1IV_P1IFG7:
      //set events for flags
      ctl_events_set_clear(&SUB_events,SUB_EV_INT_7,0);
    return;
  }
  //unknown interrupt
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
void task_tick(void) __ctl_interrupt[TIMER1_A0_VECTOR]{
  extern ticker ticker_time;
  //set rate to 1024Hz
  TA1CCR0+=32;
  //update ticker time
  ticker_time++;
  //increment timer
  ctl_increment_tick_from_isr();

  if(async_timer){
    async_timer--;
    if(!async_timer){
      ctl_events_set_clear(&BUS_helper_events,BUS_HELPER_EV_ASYNC_TIMEOUT,0);
    }
  }
  BUS_timer_timeout_check();
}

//================[System NMI Interrupt]=========================
void SYS_NMI(void)__ctl_interrupt[SYSNMI_VECTOR]{
  switch(SYSSNIV){
    //core supply voltage monitor interrupt
    case SYSSNIV_SVMLIFG:
      //event to report error
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_SVML,0);
    break;
    //input supply voltage monitor interrupt
    case SYSSNIV_SVMHIFG:
      //event to report error
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_SVMH,0);
    break;
    //core supply voltage monitor delay interrupt
    case SYSSNIV_DLYLIFG:
    break;
    //interrupt supply voltage monitor delay interrupt
    case SYSSNIV_DLYHIFG:
    break;
    //Vacant memory access interrupt
    case SYSSNIV_VMAIFG:
    break;
    //JTAG mailbox in interrupt
    case SYSSNIV_JMBINIFG:
    break;
    //JTAG mailbox out interrupt
    case SYSSNIV_JMBOUTIFG:
    break;
    //SVMLVLRIFGSVMHVLRIFG
    case SYSSNIV_VLRLIFG:
      //clear interrupt flag bits
      //unlock PMM
      PMMCTL0_H=PMMPW_H;
      //clear interrupt flags
      PMMIFG&=~(SVMLIFG|SVMLVLRIFG);
      //setup interrupt enables
      PMMRIE|=SVMLIE|SVMHIE|SVMHVLRIE|SVMLVLRIE;
      //lock PMM
      PMMCTL0_H=0;
    break;
    //SVMHVLRIFGSVMHVLRIFG
    case SYSSNIV_VLRHIFG:
      //clear interrupt flag bits
      //unlock PMM
      PMMCTL0_H=PMMPW_H;
      //clear interrupt flags
      PMMIFG&=~(SVMHIFG|SVMHVLRIFG);
      //setup interrupt enables
      PMMRIE|=SVMLIE|SVMHIE|SVMHVLRIE|SVMLVLRIE;
      //lock PMM
      PMMCTL0_H=0;
    break;
  }
}



