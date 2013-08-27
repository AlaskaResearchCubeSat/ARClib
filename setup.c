#include <ctl.h>
#include <msp430.h>
#include <stdlib.h>
#include "timerA.h"
#include "ARCbus.h"
#include "crc.h"
#include "spi.h"
#include "DMA.h"
#include "ARCbus_internal.h"
//needed to access reset error
#include "Magic.h"

//record error function, used to save an error without it cluttering up the terminal
//use the unprotected version because we are in startup code
//time ticker is not running and does not mean much at this point anyway so use a fixed zero to indicate startup errors
void _record_error(unsigned char level,unsigned short source,int err, unsigned short argument,ticker time);

//=============[initialization commands]=============
 
 //Check calibration data in segment A
int checkSegA_cal_data(void){
  unsigned short *ptr=(unsigned short*)(TLV_CHECKSUM_+2);
  unsigned short check=0;
  do{
    check^=*ptr++;
  }while(ptr<(unsigned short*)0x10FF);
  //add checksum value
  check+=TLV_CHECKSUM;
  //return result
  return check;
}
 
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

  if(checkSegA_cal_data()==0 && TLV_DCO_30_TAG==TAG_DCO_30 && TLV_DCO_30_LEN==0x08){
    //set DCO to 16MHz from calibrated values in flash
    DCOCTL=0;
    BCSCTL1=CALBC1_16MHZ;
    DCOCTL=CALDCO_16MHZ;
  }else{
    _record_error(ERR_LEV_CRITICAL,BUS_ERR_SRC_SETUP,SETUP_ERR_DCO_MISSING_CAL,0,0);
    //attempt to use a reasonable default value
    //BCSCTL1=XT2OFF|RSEL_14;
    BCSCTL1=XT2OFF|RSEL1|RSEL2|RSEL3;
    //DCOCTL=DCO_3|MOD_14;
    DCOCTL=DCO0|DCO1|MOD1|MOD2|MOD3;
  }

  //Source Mclk and SMclk from DCO (default)
  BCSCTL2=SELM_0|DIVM_0|DIVS_0;
  
  //also initialize timing generator for flash memory
  FCTL2=FWKEY|FSSEL_2|33;
  
  //set time ticker to zero
  ticker_time=0;
  //TODO: Maybe wait for LFXT to startup?
}
  
//initialize the MSP430 Clocks for low voltage operation
void initCLK_lv(void){
  extern ticker ticker_time;
  //set XT1 load caps, do this first so XT1 starts up sooner
  BCSCTL3=XCAP_0;
  //kick watchdog
  WDT_KICK();
  //setup clocks

  //set DCO values to Defalut values ~1MHz
  //TODO: decide what is best here for opperation down to 1.8V
  DCOCTL=DCO0|DCO1;
  BCSCTL1=XT2OFF|RSEL2|RSEL1|RSEL0;

  //Source Mclk and SMclk from DCO (default)
  BCSCTL2=SELM_0|DIVM_0|DIVS_0;
  
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

void initSVS(void){
  //clear SVS bits to trigger power up delay if VLD!=0
  SVSCTL=0;
  //setup SVS to trigger on 3.3V and generate a POR
  SVSCTL=VLD3|VLD1|PORON;
  //wait for SVS to power up
  //TODO: perhaps count loops to make sure we don't get stuck here
  while(!(SVSCTL&SVSON));
}

//low level setup code
void ARC_setup(void){
  //setup error reporting library
  error_init();
  //record reset error first so that it appears first in error log
  //check for reset error
  if(saved_error.magic==RESET_MAGIC_POST){
    _record_error(saved_error.level,saved_error.source,saved_error.err,saved_error.argument,0);
    //clear magic so we are not confused in the future
    saved_error.magic=RESET_MAGIC_EMPTY;
  } 
  //setup SVS
  initSVS();
  //setup clocks
  initCLK();
  //setup timerA
  init_timerA();
  //set timer to increment by 1
  ctl_time_increment=1;  
  

  //init buffer
  BUS_init_buffer();

  //TODO: determine if ctl_timeslice_period should be set to allow preemptive rescheduling
  
  //kick watchdog
  WDT_KICK();
}

//low level setup code
void ARC_setup_lv(void){
  //setup clocks
  initCLK_lv();
  //setup timerA
  init_timerA();
  //set timer to increment by 1
  ctl_time_increment=1;  
  

  //init buffer
  BUS_init_buffer();

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

//task structure idle task
extern CTL_TASK_t idle_task;

void initARCbus(unsigned char addr){
  //kick watchdog
  WDT_KICK();
  //===[initialize globals]===
  //init event sets
  ctl_events_init(&arcBus_stat.events,0);     //bus events
  ctl_events_init(&SUB_events,0);             //subsystem events
  ctl_events_init(&arcBus_stat.PortEvents,0);
  ctl_events_init(&DMA_events,0);
  //I2C mutex init
  ctl_mutex_init(&arcBus_stat.i2c_stat.mutex);
  //set I2C to idle mode
  arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
  //set SPI to idle mode
  arcBus_stat.spi_stat.mode=BUS_SPI_IDLE;
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
  //P1IES=0x00;
  //falling edge
  P1IES=0xFF;
  
  
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
