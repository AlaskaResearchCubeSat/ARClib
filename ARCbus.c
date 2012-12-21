#include <ctl.h>
#include <msp430.h>
#include <stdlib.h>
#include "timerA.h"
#include "ARCbus.h"
#include "crc.h"
#include "spi.h"

//for testing
#include <stdio.h>

//flags for internal BUS events
enum{BUS_INT_EV_I2C_CMD_RX=(1<<0),BUS_INT_EV_SPI_COMPLETE=(1<<1)};

//all events for ARCBUS internal commands
#define BUS_INT_EV_ALL    (BUS_INT_EV_I2C_CMD_RX|BUS_INT_EV_SPI_COMPLETE)

//task structure for idle task and ARC bus task
CTL_TASK_t idle_task,ARC_bus_task;

//become master on the I2C bus and receive data
short BUS_i2c_tx(unsigned short addr,const unsigned char *dat,unsigned short len);
//become master on the I2C bus and transmit txLen bytes then recive rxlen bytes
short BUS_i2c_txrx(unsigned short addr,const unsigned char *tx,unsigned short txLen,unsigned char *rx,unsigned short rxLen);

//stack for ARC bus task
unsigned BUS_stack[256];

BUS_STAT arcBus_stat;

//buffer for ISR command receive
unsigned char i2c_buf[40];

//power status
unsigned short powerState=SUB_PWR_OFF;


//task structures
CTL_TASK_t ARC_bus_task;

//buffer for SPI transactions
//TODO: make this buffer available for other uses
unsigned char SPI_buf[2048+2];

//bus internal events
static CTL_EVENT_SET_t BUS_INT_events;
//events for subsystems
CTL_EVENT_SET_t SUB_events;

//=============[initialization commands]=============
 
 //initialize the MSP430 Clocks
void initCLK(void){
  extern ticker ticker_time;
  //set XT1 load caps, do this first so XT1 starts up sooner
  BCSCTL3=XCAP_0;
  //stop watchdog
  //WDT_STOP();
  //kick watchdog
  WDT_KICK();
  //setup clocks

  //set DCO to 16MHz from calibrated values in flash
  //TODO: check to see that values are valid before using
  DCOCTL=0;
  BCSCTL1=CALBC1_16MHZ;
  DCOCTL=CALDCO_16MHZ;

  //Source Mclk and SMclk from DCO (default)
  BCSCTL2=SELM_0|DIVM_0|DIVS_0;
  
  //also initialize timing generator for flash memory
  FCTL2=FWKEY|FSSEL_2|33;

  //TODO: change for production code
  //set port 5 to output clocks
  P5DIR=BIT4|BIT5|BIT6;
  P5SEL=BIT4|BIT5|BIT6;
  
  //set time ticker to zero
  ticker_time=0;
  //TODO: Maybe wait for LFXT to startup?
}
  

//setup timer A to run off 32.768kHz xtal
void init_timerA(void){
  //setup timer A 
  TACTL=TASSEL_1|ID_0|TACLR;
  //init CCR0 for tick interrupt
  TACCR0=32;
  TACCTL0=CCIE;
}

//start timer A in continuous mode
void start_timerA(void){
//start timer A
  TACTL|=MC_2;
}

//low level setup code
void ARC_setup(void){
  //setup clocks
  initCLK();
  //setup timerA
  init_timerA();
  //set timer to increment by 1
  ctl_time_increment=1;  
  
  //TODO: determine if ctl_timeslice_period should be set to allow preemptive rescheduling
  
  //kick watchdog
  WDT_KICK();
}

//TODO: determine if these are necessary at startup
//generate a clock on the I2C bus
//clock frequency is about 10kHz
void I2C_clk(void){
  //pull clock line low
  P3DIR|=BIT2;
  //wait for 0.05ms
  __delay_cycles(800);
  //realese clock line
  P3DIR&=~BIT2;
  //wait for 0.05ms
  __delay_cycles(800);
}

//force a reset on the I2C bus if SDA is stuck low
void I2C_reset(void){
  int i;
  //clock and data pins as GPIO
  P3SEL&=~(BIT1|BIT2);
  //clock and data pins as inputs
  P3DIR&=~(BIT1|BIT2);
  //set out bits to zero
  P3OUT&=~(BIT1|BIT2);
  //check if SDA is stuck low
  if(!(P3IN&BIT1)){
    //generate 9 clocks for good measure
    for(i=0;i<9;i++){
      I2C_clk();
    }
    //pull SDA low 
    P3DIR|=BIT1;
    //wait for 0.05ms
    __delay_cycles(800);
    //pull SCL low
    P3DIR|=BIT2;
    //wait for 0.05ms
    __delay_cycles(800);
    //realese SCL
    P3DIR&=~BIT2;
    //wait for 0.05ms
    __delay_cycles(800);
    //realese SDA
    P3DIR&=~BIT1;
  }
  //clock and data pins as I2C function
  P3SEL|=BIT1|BIT2;
}

void initARCbus(unsigned char addr){
  //kick watchdog
  WDT_KICK();
  //===[initialize globals]===
  //init event sets
  ctl_events_init(&arcBus_stat.events,0);     //bus events
  ctl_events_init(&BUS_INT_events,0);         //internal events
  ctl_events_init(&SUB_events,0);             //subsystem events
  ctl_events_init(&arcBus_stat.PortEvents,0);
  //set I2C to idle mode
  arcBus_stat.i2c_stat.mode=I2C_IDLE;
  //set SPI to idle mode
  arcBus_stat.spi_stat.mode=SPI_IDLE;
  //startup with power off
  powerState=SUB_PWR_OFF;
  //============[setup I2C]============ 
  //put UCB0 into reset state
  UCB0CTL1=UCSWRST;
  //setup registers
  //UCB0CTL0=UCMM|UCMODE_3|UCSYNC;
  UCB0CTL0=UCMM|UCMST|UCMODE_3|UCSYNC;
  UCB0CTL1|=UCSSEL_2;
  //set baud rate to 400kB/s off of 16MHz SMCLK
  //UCB0BR0=0x28;
  //UCB0BR1=0x00;
  //set baud rate to 100kB/s off of 16MHz SMCLK
  //UCB0BR0=0xA0;
  //UCB0BR1=0x00;
  //set baud rate to 50kB/s off of 16MHz SMCLK
  UCB0BR0=0x40;
  UCB0BR1=0x01;
  //set baud rate to 1kB/s off of 16MHz SMCLK
  //UCB0BR0=0x80;
  //UCB0BR1=0x3E;
  //set own address
  UCB0I2COA=UCGCEN|addr;
  //configure ports
  P3SEL|=BIT1|BIT2;
  //bring UCB0 out of reset state
  UCB0CTL1&=~UCSWRST;
  //enable state change interrupts
  UCB0I2CIE|=UCNACKIE|UCSTTIE|UCALIE;
  //enable Tx and Rx interrupts
  //UC0IE|=UCB0RXIE|UCB0TXIE;
  //============[setup SPI]============
  //put UCA0 into reset state
  UCA0CTL1=UCSWRST;
  //set MSB first, 3 wire SPI mod, 8 bit words
  UCA0CTL0=UCMSB|UCMODE_0|UCSYNC;
  //clock UCA0 off of SMCLK
  UCA0CTL1|=UCSSEL_2;
  //set SPI clock to 3.2MHz
  UCA0BR0=0x05;
  UCA0BR1=0;
  //set SPI clock to 1MHz
  //UCA0BR0=0x10;
  //UCA0BR1=0;
  //set SPI clock to 250kHz
  //UCA0BR0=0x40;
  //UCA0BR1=0;
  //leave UCA1 in reset state until it is used for communication
    
  //======[setup pin interrupts]=======

  //rising edge
  //P2IES=0x00;
  //falling edge
  P2IES=0xFF;
  
  
  #ifdef CDH_LIB
    //pull down resistors
    //P1OUT&=~(BIT0|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
    //pull up resistors
    P1OUT=0xFF;
    //enable pull resistors
    P1REN=0xFF;
  #else
    //disable pullups
    P1REN=0;
  #endif
  
  //clear flags
  P1IFG=0;
  //enable interrupts
  P1IE|=(BIT0|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);

   //create a main task with maximum priority so other tasks can be created without interruption
  //this should be called before other tasks are created
  ctl_task_init(&idle_task, 255, "idle");  

  //start timerA
  start_timerA();

}

//ARC bus Task, do ARC bus stuff
static void ARC_bus_run(void *p) __toplevel{
  unsigned int e;
  unsigned char len;
  unsigned char addr,resp,cmd;
  unsigned char pk[40];
  unsigned char *ptr;
  unsigned short crc;
  //address of SPI slave during transaction
  unsigned char SPI_addr=0;
  ticker nt;
  int snd,i;
  //first send "I'm on" command
  BUS_cmd_init(pk,CMD_SUB_POWERUP);//setup command
  //send command
  resp=BUS_cmd_tx(BUS_ADDR_CDH,pk,0,0,SEND_FOREGROUND);
  #ifndef CDH_LIB         //Subsystem board 
    //check for failed send
    if(resp!=RET_SUCCESS){
      //wait a bit
      ctl_timeout_wait(ctl_get_current_time()+30);
      //resend
      resp=BUS_cmd_tx(BUS_ADDR_CDH,pk,0,0,SEND_FOREGROUND);
      //check for success
      if(resp!=RET_SUCCESS){
        //Failed
        puts("Failed to detect CDH board\r");
      }
    }
  #else     //CDH board, check for other CDH board
    if(resp==RET_SUCCESS){
      puts("Other CDH board detected.\r");
      //TODO : this is bad. perhaps do something here to recover
    }
  #endif
  //event loop
  for(;;){
    //wait for something to happen
    e = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&BUS_INT_events,BUS_INT_EV_ALL,CTL_TIMEOUT_NONE,0);
    if(e&BUS_INT_EV_SPI_COMPLETE){
      //check if SPI was in progress
      if(SPI_addr){
        //turn off SPI
        SPI_deactivate();
        //done with SPI send command
        BUS_cmd_init(pk,CMD_SPI_COMPLETE);
        BUS_cmd_tx(SPI_addr,pk,0,0,SEND_BGND);
        //transaction complete, clear address
        SPI_addr=0;
        //assemble CRC
        crc=SPI_buf[arcBus_stat.spi_stat.len+1];//LSB
        crc|=(((unsigned short)SPI_buf[arcBus_stat.spi_stat.len])<<8);//MSB
        //check CRC
        if(crc!=crc16(SPI_buf,arcBus_stat.spi_stat.len)){
          //Bad CRC
          ctl_events_set_clear(&SUB_events,SUB_EV_SPI_ERR_CRC,0);
        }else{
          //tell subsystem, SPI data received
          ctl_events_set_clear(&SUB_events,SUB_EV_SPI_DAT,0);
        }
      }
    }
    if(e&BUS_INT_EV_I2C_CMD_RX){
      //=====================[I2C Command Received, Process command]===============================
      //clear response
      resp=0;
      //get len
      len=arcBus_stat.i2c_stat.rx.idx;
      //compute crc
      crc=crc8(i2c_buf,len-1);
      //get length of payload
      len=len-I2C_CRC_LEN-I2C_HDR_LEN;
      //get sender address
      addr=CMD_ADDR_MASK&i2c_buf[0];
      //get command type
      cmd=i2c_buf[1];
      //point to the first payload byte
      ptr=&i2c_buf[2];
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
            //cause a PUC by writing the wrong password to the watchdog
            WDT_RESET();
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
            if(arcBus_stat.spi_stat.len+2>sizeof(SPI_buf)){
              //length is too long
              //cause NACK to be sent
              resp=ERR_SPI_LEN;
              break;
            }
            //TODO: give error if already sending
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
            DMA0SZ = arcBus_stat.spi_stat.len+SPI_CRC_LEN;
            // Configure the DMA transfer, single byte transfer with destination increment
            DMA0CTL = DMAIE|DMADT_0|DMASBDB|DMAEN|DMADSTINCR1|DMADSTINCR0;
            // Source DMA address: tx buffer
            //skip the second byte, first byte sent manually
            DMA1SA = (unsigned int)(arcBus_stat.spi_stat.tx+1);
            // Destination DMA address: the transmit buffer.
            DMA1DA = (unsigned int)(&UCA0TXBUF);
            // The size of the block to be transferred
            DMA1SZ = arcBus_stat.spi_stat.len+SPI_CRC_LEN-1;
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
            BUS_cmd_init(pk,CMD_SPI_CLEAR);
            BUS_cmd_tx(BUS_ADDR_CDH,pk,0,0,SEND_BGND);
#endif
            //notify calling task
            ctl_events_set_clear(&arcBus_stat.events,BUS_EV_SPI_COMPLETE,0);
          break;

          case CMD_NACK:
            //TODO: handle this better somehow?
            //set event 
            ctl_events_set_clear(&arcBus_stat.events,BUS_EV_CMD_NACK,0);
            //TESTING: print message
            printf("\r\nNACK recived for command %i with reason %i\r\n",ptr[0],ptr[1]);
          break;
          default:
            //check for subsystem command
            resp=SUB_parseCmd(addr,cmd,ptr,len);
          break;
        }
        //malformed command, send nack if requested
        if(resp!=0 && i2c_buf[0]&CMD_TX_NACK){
          printf("Error : resp %i\r\n",resp);
          //setup command
          ptr=BUS_cmd_init(pk,CMD_NACK);
          //send NACK reason
          ptr[0]=resp;
          //send packet
          BUS_cmd_tx(addr,pk,1,0,SEND_BGND);
        }
      }else if(cmd!=CMD_NACK){
        //CRC check failed, send NACK
        printf("Error : bad CRC Command %i\r\nRecived CRC 0x%02X calculated CRC 0x%02X\r\n",cmd,ptr[len],crc);
        //print out packet
        for(i=0;i<len;i++){
          printf("0x%02X ",ptr[i]);
        }
        //setup command
        ptr=BUS_cmd_init(pk,CMD_NACK);
        //send NACK reason
        ptr[0]=ERR_BAD_CRC;
        //send packet
        BUS_cmd_tx(addr,pk,1,0,SEND_BGND);
      }
    }
  }
}

//=======================================================================================
//                                 [Main Loop]
//=======================================================================================

//main loop function, start ARC_Bus task then enter Idle task
void mainLoop(void) __toplevel{
  //start ARCbus task
  ctl_task_run(&ARC_bus_task,20,ARC_bus_run,NULL,"ARC_Bus",sizeof(BUS_stack)/sizeof(BUS_stack[0])-2,BUS_stack+1,0);
  // drop to lowest priority to start created tasks running.
  ctl_task_set_priority(&idle_task,0); 
  
  //main idle loop
  //NOTE that this task should never wait to ensure that there is always a runnable task
  for(;;){    
      //kick watchdog
      WDT_KICK();
      //go to low power mode
      LPM0;
  }
}

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
        arcBus_stat.i2c_stat.mode=I2C_IDLE;
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
        UCB0TXBUF=I2C_DUMMY_DATA;
      }else{//Master Mode
        //generate stop condition
        UCB0CTL1|=UCTXSTP;
        //set event
        ctl_events_set_clear(&arcBus_stat.events,BUS_EV_I2C_COMPLETE,0);
        //set state to idle
        arcBus_stat.i2c_stat.mode=I2C_IDLE;
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
    //set ERROR flag
    ctl_events_set_clear(&arcBus_stat.events,BUS_EV_I2C_NACK,0);
    //set state to idle
    arcBus_stat.i2c_stat.mode=I2C_IDLE;
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
    if(arcBus_stat.i2c_stat.mode==I2C_RX){
      //set flag to notify 
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_I2C_CMD_RX,0);
    }
    //set state to idle
    arcBus_stat.i2c_stat.mode=I2C_IDLE;
    //disable I2C Tx and Rx Interrupts
    UC0IE&=~(UCB0TXIE|UCB0RXIE);
    //disable stop interrupt
    UCB0I2CIE&=~UCSTPIE;
  }
  //start condition and slave address received, setup for command
  if(UCB0STAT&UCSTTIFG){
    //check status
    //This is to fix the issue where the start condition happens before the stop can be processed
    if(arcBus_stat.i2c_stat.mode==I2C_RX){
      //set flag to notify 
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_I2C_CMD_RX,0);
      //set state to idle
      arcBus_stat.i2c_stat.mode=I2C_IDLE;
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
      arcBus_stat.i2c_stat.mode=I2C_TX;
      //send first byte to save time
      UCB0TXBUF=I2C_DUMMY_DATA;
    }else{
      //enable I2C Rx Interrupt
      UC0IE|=UCB0RXIE;
      //setup receive status
      arcBus_stat.i2c_stat.rx.ptr=i2c_buf;
      arcBus_stat.i2c_stat.rx.len=sizeof(i2c_buf);
      arcBus_stat.i2c_stat.rx.idx=0;
      //set mode to Rx
      arcBus_stat.i2c_stat.mode=I2C_RX;
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
    arcBus_stat.i2c_stat.mode=I2C_IDLE;
  }
}

//=================[Port pin Handler]=============================
void bus_int(void) __ctl_interrupt[PORT1_VECTOR]{
  unsigned char flags=P1IFG&P1IE;
  P1IFG&=~flags;
  //set events for flags
  ctl_events_set_clear(&arcBus_stat.PortEvents,flags,0);
  //TESTING: send time check event
  ctl_events_set_clear(&SUB_events,SUB_EV_TIME_CHECK,0);
}


//================[DMA Transfer Complete]=========================
void DMA_int(void) __ctl_interrupt[DMA_VECTOR]{
  switch(DMAIV){
    case DMAIV_DMA0IFG:
      ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_SPI_COMPLETE,0);
    break;
    case DMAIV_DMA1IFG:
      //ctl_events_set_clear(&arcBus_stat.events,BUS_EV_SPI_COMPLETE,0);
    break;
    case DMAIV_DMA2IFG:
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
}
