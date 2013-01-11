#include <ctl.h>
#include <msp430.h>
#include <stdlib.h>
#include "timerA.h"
#include "ARCbus.h"
#include "crc.h"
#include "spi.h"


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
  //P5DIR=BIT4|BIT5|BIT6;
  //P5SEL=BIT4|BIT5|BIT6;
  
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
