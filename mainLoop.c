#include <ctl.h>
#include <msp430.h>
#include <stdlib.h>
#include "timerA.h"
#include "ARCbus.h"
#include "crc.h"
#include "spi.h"
#include <Error.h>
#include "ARCbus_internal.h"

//record error function, used to save an error without it cluttering up the terminal
void record_error(unsigned char level,unsigned short source,int err, unsigned short argument,ticker time);

//bus internal events
CTL_EVENT_SET_t BUS_INT_events,BUS_helper_events;

//task structure for idle task and ARC bus task
CTL_TASK_t idle_task,ARC_bus_task,ARC_bus_helper_task;

//stack for ARC bus task
unsigned BUS_stack[256],helper_stack[250];

BUS_STAT arcBus_stat;

//events for subsystems
CTL_EVENT_SET_t SUB_events;

//address of SPI slave during transaction
static unsigned char SPI_addr=0;

static void ARC_bus_helper(void *p);

//ARC bus Task, do ARC bus stuff
static void ARC_bus_run(void *p) __toplevel{
  unsigned int e;
  unsigned char len;
  unsigned char addr,cmd;
  int resp;
  unsigned char pk[40];
  unsigned char *ptr;
  unsigned short crc;
  unsigned char *SPI_buf=NULL;
  int state;
  ticker nt;
  int snd,i;
  SPI_addr=0;
  //Initialize ErrorLib
  error_recording_start();
  //first send "I'm on" command
  BUS_cmd_init(pk,CMD_SUB_POWERUP);//setup command
  //send command
  resp=BUS_cmd_tx(BUS_ADDR_CDH,pk,0,0,BUS_I2C_SEND_FOREGROUND);
  #ifndef CDH_LIB         //Subsystem board 
    //check for failed send
    if(resp!=RET_SUCCESS){
      //give a warning
      report_error(ERR_LEV_WARNING,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_CDH_NOT_FOUND,resp);
      //wait a bit
      ctl_timeout_wait(ctl_get_current_time()+30);
      //resend
      resp=BUS_cmd_tx(BUS_ADDR_CDH,pk,0,0,BUS_I2C_SEND_FOREGROUND);
      //check for success
      if(resp!=RET_SUCCESS){
        //Failed
        report_error(ERR_LEV_ERROR,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_CDH_NOT_FOUND,resp);     
      }
    }
  #else     //CDH board, check for other CDH board
    if(resp==RET_SUCCESS){
      report_error(ERR_LEV_ERROR+30,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_MUTIPLE_CDH,0);
      //TODO : this is bad. perhaps do something here to recover
    }
  #endif
  //initialize helper events
  ctl_events_init(&BUS_helper_events,0);
  //start helper task
  ctl_task_run(&ARC_bus_helper_task,BUS_PRI_ARCBUS_HELPER,ARC_bus_helper,NULL,"ARC_Bus_helper",sizeof(helper_stack)/sizeof(helper_stack[0])-2,helper_stack+1,0);
  
  //event loop
  for(;;){
    //wait for something to happen
    e = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&BUS_INT_events,BUS_INT_EV_ALL,CTL_TIMEOUT_NONE,0);
    //check if buffer can be unlocked
    if(e&BUS_INT_EV_BUFF_UNLOCK){
      SPI_buf=NULL;
      //unlock buffer
      BUS_free_buffer();
    }
    //check if I2C mutex can be released
    if(e&BUS_INT_EV_RELEASE_MUTEX){
      //release I2C mutex
      BUS_I2C_release();
    }
    //check if a SPI transaction is complete
    if(e&BUS_INT_EV_SPI_COMPLETE){
      //check if SPI was in progress
      if(SPI_addr){
        //turn off SPI
        SPI_deactivate();
        //tell helper thread to send SPI complete command
        ctl_events_set_clear(&BUS_helper_events,BUS_HELPER_EV_SPI_COMPLETE_CMD,0);
        //transaction complete, clear address
        SPI_addr=0;
        //assemble CRC
        crc=SPI_buf[arcBus_stat.spi_stat.len+1];//LSB
        crc|=(((unsigned short)SPI_buf[arcBus_stat.spi_stat.len])<<8);//MSB
        //check CRC
        if(crc!=crc16(SPI_buf,arcBus_stat.spi_stat.len)){
          //Bad CRC
          //clear buffer pointer
          SPI_buf=NULL;
          //free buffer
          BUS_free_buffer();
          //send event
          ctl_events_set_clear(&SUB_events,SUB_EV_SPI_ERR_CRC,0);
        }else{
          //tell subsystem, SPI data received
          //Subsystem must signal to free the buffer
          ctl_events_set_clear(&SUB_events,SUB_EV_SPI_DAT,0);
        }
      }
    }
    //check if an I2C command has been received
    if(e&BUS_INT_EV_I2C_CMD_RX){
        //check if packet is complete
        if(I2C_rx_buf[I2C_rx_out].stat!=I2C_PACKET_STAT_COMPLETE){
          report_error(ERR_LEV_ERROR,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_RX_BUF_STAT,I2C_rx_buf[I2C_rx_out].stat);
        }else{
        //clear response
        resp=0;
        //get len
        len=I2C_rx_buf[I2C_rx_out].len;
        //compute crc
        crc=crc8(I2C_rx_buf[I2C_rx_out].dat,len-1);
        //get length of payload
        len=len-BUS_I2C_CRC_LEN-BUS_I2C_HDR_LEN;
        //get sender address
        addr=CMD_ADDR_MASK&I2C_rx_buf[I2C_rx_out].dat[0];
        //get command type
        cmd=I2C_rx_buf[I2C_rx_out].dat[1];
        //point to the first payload byte
        ptr=&I2C_rx_buf[I2C_rx_out].dat[2];
        //check crc for packet
        if(ptr[len]==crc){
          //handle command based on command type
          switch(cmd){
            case CMD_SUB_ON:            
                //check for proper length
                if(len!=0){
                  resp=ERR_PK_LEN;
                }
                //set new power status
                powerState=SUB_PWR_ON;
                //inform subsystem
                ctl_events_set_clear(&SUB_events,SUB_EV_PWR_ON,0);
            break;
            case CMD_SUB_OFF:
              //check to make sure that the command is directed to this subsystem
              if(len==1 && ptr[0]==UCB0I2COA){
                //set new power status
                powerState=SUB_PWR_ON;
                //inform subsystem
                ctl_events_set_clear(&SUB_events,SUB_EV_PWR_OFF,0);
              }else{
                //error with command
                resp=ERR_BAD_PK;
              }
            break;
            case CMD_SUB_STAT:
              //check for proper length
              if(len!=4){
                resp=ERR_PK_LEN;
              }
              //assemble time from packet
              nt=ptr[3];
              nt|=((ticker)ptr[2])<<8;
              nt|=((ticker)ptr[1])<<16;
              nt|=((ticker)ptr[0])<<24;
              //update time
              set_ticker_time(nt);
              //tell subsystem to send status
              ctl_events_set_clear(&SUB_events,SUB_EV_SEND_STAT,0);
            break;
            case CMD_RESET:          
              //check for proper length
              if(len!=0){
                resp=ERR_PK_LEN;
              }
              //reset msp430
              reset(ERR_LEV_INFO,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_RESET,0);
              //TODO: code should never get here, handle this if it does happen
              break;
            case CMD_SPI_RDY:
              //check length
              if(len!=2){
                resp=ERR_PK_LEN;
                break;
              }
              //assemble length
              arcBus_stat.spi_stat.len=ptr[1];//LSB
              arcBus_stat.spi_stat.len|=(((unsigned short)ptr[0])<<8);//MSB
              //check length account for 16bit CRC
              if(arcBus_stat.spi_stat.len+2>BUS_get_buffer_size()){
                //length is too long
                //cause NACK to be sent
                resp=ERR_SPI_LEN;
                break;
              }
              //check if already transmitting
              if(SPI_buf!=NULL){
                resp=ERR_SPI_BUSY;
                break;
              }
              SPI_buf=BUS_get_buffer(CTL_TIMEOUT_NOW,0);
              //check if buffer was locked
              if(SPI_buf==NULL){
                resp=ERR_BUFFER_BUSY;
                break;
              }
              //save address of SPI slave
              SPI_addr=addr;
              //TODO : Fill with actual data
              //fill  buffer with "random" data
              for(i=0;i<arcBus_stat.spi_stat.len;i++){
                SPI_buf[i]=i;
              }
              //calculate CRC
              crc=crc16(SPI_buf,arcBus_stat.spi_stat.len);
              //send CRC in big endian order
              SPI_buf[arcBus_stat.spi_stat.len]=crc>>8;
              SPI_buf[arcBus_stat.spi_stat.len+1]=crc;
              //setup SPI structure
              arcBus_stat.spi_stat.rx=SPI_buf;
              arcBus_stat.spi_stat.tx=SPI_buf;
              //Setup SPI bus to exchange data as master
              SPI_master_setup();
              //============[setup DMA for transfer]============
              //setup source trigger
              DMACTL0 &=~(DMA0TSEL_15|DMA1TSEL_15);
              DMACTL0 |= (DMA0TSEL_3|DMA1TSEL_4);
              // Source DMA address: receive register.
              DMA0SA = (unsigned short)(&UCA0RXBUF);
              // Destination DMA address: rx buffer.
              DMA0DA = (unsigned short)SPI_buf;
              // The size of the block to be transferred
              DMA0SZ = arcBus_stat.spi_stat.len+BUS_SPI_CRC_LEN;
              // Configure the DMA transfer, single byte transfer with destination increment
              DMA0CTL = DMAIE|DMADT_0|DMASBDB|DMAEN|DMADSTINCR1|DMADSTINCR0;
              // Source DMA address: tx buffer
              //skip the second byte, first byte sent manually
              DMA1SA = (unsigned int)(arcBus_stat.spi_stat.tx+1);
              // Destination DMA address: the transmit buffer.
              DMA1DA = (unsigned int)(&UCA0TXBUF);
              // The size of the block to be transferred
              DMA1SZ = arcBus_stat.spi_stat.len+BUS_SPI_CRC_LEN-1;
              // Configure the DMA transfer, single byte transfer with source increment
              DMA1CTL=DMADT_0|DMASBDB|DMAEN|DMASRCINCR1|DMASRCINCR0;
              //write first byte into the Tx buffer to start transfer
              UCA0TXBUF=*arcBus_stat.spi_stat.tx;
            break;
            
            case CMD_SPI_COMPLETE:
              //turn off SPI
              SPI_deactivate();
              //SPI transfer is done
              //notify CDH board
              //TODO: some sort of error check to make sure that a SPI transfer was requested and send an ERROR if one was not
#ifndef CDH_LIB
              ctl_events_set_clear(&BUS_helper_events,BUS_HELPER_EV_SPI_CLEAR_CMD,0);
#endif
              //notify calling task
              ctl_events_set_clear(&arcBus_stat.events,BUS_EV_SPI_COMPLETE,0);
            break;
            case CMD_ASYNC_SETUP:
              //check length
              if(len!=1){
                resp=ERR_PK_LEN;
                break;
              }
              switch(ptr[0]){
                case ASYNC_OPEN:
                  //open remote connection
                  async_open_remote(addr);
                break;
                case ASYNC_CLOSE:
                  //check if sending address corosponds to async address
                  if(async_addr!=addr){
                    //report error
                    report_error(ERR_LEV_ERROR,BUS_ERR_SRC_ASYNC,ASYNC_ERR_CLOSE_WRONG_ADDR,(((unsigned short)addr)<<8)|async_addr);
                    break;
                  }
                  //tell helper thread to close connection
                  ctl_events_set_clear(&BUS_helper_events,BUS_HELPER_EV_ASYNC_CLOSE,0);
                break;
              }
            break;
            case CMD_ASYNC_DAT:
              //post bytes to queue
              ctl_byte_queue_post_multi_nb(&async_rxQ,len,ptr);
            break;
            case CMD_NACK:
              //TODO: handle this better somehow?
              //set event 
              ctl_events_set_clear(&arcBus_stat.events,BUS_EV_CMD_NACK,0);
              //report error
              report_error(ERR_LEV_ERROR,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_NACK_REC,(((unsigned short)ptr[0])<<8)|((unsigned short)ptr[1]));
            break;
            default:
              //check for subsystem command
              resp=SUB_parseCmd(addr,cmd,ptr,len);
            break;
          }
          //malformed command, send nack if requested
          if(resp!=0 && I2C_rx_buf[I2C_rx_out].dat[0]&CMD_TX_NACK){
            report_error(ERR_LEV_ERROR,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_BAD_CMD,(((unsigned short)resp)<<8)|((unsigned short)cmd));
            //setup command
            ptr=BUS_cmd_init(pk,CMD_NACK);
            //send NACK reason
            ptr[0]=resp;
            //send packet
            BUS_cmd_tx(addr,pk,1,0,BUS_I2C_SEND_BGND);
          }
        }else if(cmd!=CMD_NACK){
          //CRC check failed, send NACK
          report_error(ERR_LEV_ERROR,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_CMD_CRC,cmd);
          //setup command
          ptr=BUS_cmd_init(pk,CMD_NACK);
          //send NACK reason
          ptr[0]=ERR_BAD_CRC;
          //send packet
          BUS_cmd_tx(addr,pk,1,0,BUS_I2C_SEND_BGND);
        }
        //disable interrupts to modify queue indexes
        state=ctl_global_interrupts_set(0);
        //increment index
        I2C_rx_out++;
        //check for wraparound
        if(I2C_rx_out>=BUS_I2C_PACKET_QUEUE_LEN){
          I2C_rx_out=0;
        }
        //restore interrupt status
        ctl_global_interrupts_set(state);
        //done with packet set status
        I2C_rx_buf[I2C_rx_out].stat=I2C_PACKET_STAT_EMPTY;
      }
    }
    //check for errors and report
    if(e&BUS_INT_EV_I2C_RX_BUSY){
      report_error(ERR_LEV_ERROR,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_I2C_RX_BUSY,0);
    }
    if(e&BUS_INT_EV_I2C_ARB_LOST){
      report_error(ERR_LEV_DEBUG,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_I2C_ARB_LOST,0);
    }
  }
}
    
    
//ARC bus Task, do ARC bus stuff
static void ARC_bus_helper(void *p) __toplevel{
  unsigned int e;
  int resp;
  unsigned char *ptr,pk[BUS_I2C_HDR_LEN+0+BUS_I2C_CRC_LEN];
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&BUS_helper_events,BUS_HELPER_EV_ALL,CTL_TIMEOUT_NONE,0);
    //async timer timed out, send data
    if(e&BUS_HELPER_EV_ASYNC_TIMEOUT){
      //send some data
      async_send_data();
    }
    if(e&BUS_HELPER_EV_SPI_COMPLETE_CMD){      
      //done with SPI send command
      BUS_cmd_init(pk,CMD_SPI_COMPLETE);
      resp=BUS_cmd_tx(SPI_addr,pk,0,0,BUS_I2C_SEND_FOREGROUND);
      //check if command was successful and try again if it failed
      if(resp!=RET_SUCCESS){
        resp=BUS_cmd_tx(SPI_addr,pk,0,0,BUS_I2C_SEND_FOREGROUND);
      }
      //check if command sent successfully
      if(resp!=RET_SUCCESS){
        //report error
        report_error(ERR_LEV_ERROR,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_SPI_COMPLETE_FAIL,resp);
      }
    }
    if(e&BUS_HELPER_EV_SPI_CLEAR_CMD){
      //done with SPI send command
      BUS_cmd_init(pk,CMD_SPI_CLEAR);
      resp=BUS_cmd_tx(BUS_ADDR_CDH,pk,0,0,BUS_I2C_SEND_FOREGROUND);
      //check if command was successful and try again if it failed
      if(resp!=RET_SUCCESS){
        resp=BUS_cmd_tx(BUS_ADDR_CDH,pk,0,0,BUS_I2C_SEND_FOREGROUND);
      }
      //check if command sent successfully
      if(resp!=RET_SUCCESS){
        //report error
        report_error(ERR_LEV_ERROR,BUS_ERR_SRC_MAIN_LOOP,MAIN_LOOP_ERR_SPI_CLEAR_FAIL,resp);
      }
    }
    if(e&BUS_HELPER_EV_ASYNC_CLOSE){      
      //close async connection
      async_close_remote();
      //send event
      ctl_events_set_clear(&SUB_events,SUB_EV_ASYNC_CLOSE,0);
    }
  }
}

//=======================================================================================
//                                 [Main Loop]
//=======================================================================================

//main loop function, start ARC_Bus task then enter Idle task
void mainLoop(void) __toplevel{
  //initialize events
  ctl_events_init(&BUS_INT_events,0);
  //start ARCbus task
  ctl_task_run(&ARC_bus_task,BUS_PRI_ARCBUS,ARC_bus_run,NULL,"ARC_Bus",sizeof(BUS_stack)/sizeof(BUS_stack[0])-2,BUS_stack+1,0);
  //kick WDT to give us some time
  WDT_KICK();
  // drop to lowest priority to start created tasks running.
  ctl_task_set_priority(&idle_task,0); 
  
  //main idle loop
  //NOTE that this task should never wait to ensure that there is always a runnable task
  for(;;){    
      //kick watchdog
      WDT_KICK();
      #ifndef IDLE_DEBUG
        //go to low power mode
        LPM0;
      #else
        P7OUT^=BIT0+BIT1+BIT2+BIT3;
      #endif
  }
}
