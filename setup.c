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
#include "vcore.h"

//record error function, used to save an error without it cluttering up the terminal
//use the unprotected version because we are in startup code
//time ticker is not running and does not mean much at this point anyway so use a fixed zero to indicate startup errors
void _record_error(unsigned char level,unsigned short source,int err, unsigned short argument,ticker time);

//=============[initialization commands]=============
 
 
 //initialize the MSP430 Clocks
void initCLK(void){
  extern ticker ticker_time;
  //set XT1 load caps, do this first so XT1 starts up sooner
  UCSCTL6=XCAP_0|XT2OFF|XT1DRIVE_3;
  //stop watchdog
  //WDT_STOP();
  //kick watchdog
  WDT_KICK();
  //set higher core voltage
  PMM_setVCore(PMM_CORE_LEVEL_3);
  
  //setup clocks
  //set frequency range
  UCSCTL1=DCORSEL_5;
  //setup FLL for 19.99 MHz operation
  UCSCTL2=FLLD__4|(609);
  UCSCTL3=SELREF__XT1CLK|FLLREFDIV__4;
  //use XT1 for ACLK and DCO for MCLK and SMCLK
  UCSCTL4=SELA_0|SELS_3|SELM_3;

 
  
  //set time ticker to zero
  ticker_time=0;
  //TODO: Maybe wait for LFXT to startup?
}
  

//setup timer A to run off 32.768kHz xtal
void init_timerA(void){
  //setup timer A 
  TA1CTL=TASSEL_1|ID_0|TACLR;
  //init CCR0 for tick interrupt
  TA1CCR0=32;
  TA1CCTL0=CCIE;
}

//start timer A in continuous mode
void start_timerA(void){
//start timer A
  TA1CTL|=MC_2;
}

//setup Supply voltage supervisor and monitor levels and interrupts
void initSVS(void){
  //check voltage level
  switch(PMMCTL0&PMMCOREV_3){
    //settings for highest core voltage settings
    case PMMCOREV_3:
      //unlock PMM
      PMMCTL0_H=PMMPW_H;
      //setup high side supervisor and monitor
      SVSMHCTL=SVMHE|SVSHE|SVSHRVL_3|SVSMHRRL_7;
      //lock PMM
      PMMCTL0_H=0;
    break;
  }
  //setup interrupts
  PMMRIE|=SVMLIE|SVMHIE;
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
  //setup clocks
  initCLK();
  //setup SVS
  initSVS();
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
  P3SEL0&=~(BUS_PINS_I2C);
  //clock and data pins as inputs
  P3DIR&=~(BUS_PINS_I2C);
  //set out bits to zero
  P3OUT&=~(BUS_PINS_I2C);
  //check if SDA is stuck low
  if(!(P3IN&BUS_PIN_SDA)){
    //generate 9 clocks for good measure
    for(i=0;i<9;i++){
      I2C_clk();
    }
    //pull SDA low 
    P3DIR|=BUS_PIN_SDA;
    //wait for 0.05ms
    __delay_cycles(800);
    //pull SCL low
    P3DIR|=BUS_PIN_SCL;
    //wait for 0.05ms
    __delay_cycles(800);
    //realese SCL
    P3DIR&=~BUS_PIN_SCL;
    //wait for 0.05ms
    __delay_cycles(800);
    //realese SDA
    P3DIR&=~BUS_PIN_SDA;
  }
  //clock and data pins as I2C function
  P3SEL0|=BUS_PINS_I2C;
}

//task structure idle task
extern CTL_TASK_t idle_task;

void initARCbus(unsigned char addr){
  int i;
  //kick watchdog
  WDT_KICK();
  //===[initialize globals]===
  //init event sets
  ctl_events_init(&arcBus_stat.events,0);     //bus events
  ctl_events_init(&SUB_events,0);             //subsystem events
  ctl_events_init(&DMA_events,0);
  //I2C mutex init
  ctl_mutex_init(&arcBus_stat.i2c_stat.mutex);
  //set I2C to idle mode
  arcBus_stat.i2c_stat.mode=BUS_I2C_IDLE;
  //initialize I2C packet queue to empty state
  for(i=0;i<BUS_I2C_PACKET_QUEUE_LEN;i++){
    I2C_rx_buf[i].stat=I2C_PACKET_STAT_EMPTY;
  }
  //initialize I2C packet queue pointers
  I2C_rx_in=I2C_rx_out=0;
  //set SPI to idle mode
  arcBus_stat.spi_stat.mode=BUS_SPI_IDLE;
  //startup with power off
  powerState=SUB_PWR_OFF;
  //========[setup port mapping]=======
  //unlock registers
  PMAPKEYID=PMAPKEY;
  //setup BUS I2C SCL
  P3MAP0=PM_UCB0SCL;
  //setup BUS I2C SDA
  P3MAP1=PM_UCB0SDA;
  //setup BUS SPI CLK
  P3MAP2=PM_UCA0CLK;
  //setup BUS SPI SOMI
  P3MAP3=PM_UCA0SOMI;
  //setup BUS SPI SIMO
  P3MAP4=PM_UCA0SIMO;
  //set old definitions to unused
  P2MAP5=PM_NONE;
  P2MAP6=PM_NONE;
  P2MAP7=PM_NONE;
  //lock the Port map module
  //do not allow reconfiguration
  PMAPKEYID=0;
  //============[setup I2C]============ 
  //put UCB0 into reset state
  UCB0CTLW0=UCSWRST;
  //setup registers
  UCB0CTLW0|=UCMM|UCMST|UCMODE_3|UCSYNC|UCSSEL_2;
  UCB0CTLW1=UCCLTO_3|UCASTP_0|UCGLIT_0;
  //set baud rate to 400kB/s off of 16MHz SMCLK
  //UCB0BR0=0x28;
  //UCB0BR1=0x00;
  //set baud rate to 100kB/s off of 16MHz SMCLK
  //UCB0BR0=0xA0;
  //UCB0BR1=0x00;
  //set baud rate to 50kB/s off of 20MHz SMCLK
  //UCB0BRW=40;
  //set baud rate to 1kB/s off of 20MHz SMCLK
  UCB0BRW=2000;
  //set baud rate to 1kB/s off of 16MHz SMCLK
  //UCB0BR0=0x80;
  //UCB0BR1=0x3E;
  //set own address
  UCB0I2COA0=UCOAEN|addr;
  //enable general call address
  UCB0I2COA0|=UCGCEN;
  //configure ports
  P3SEL0|=BUS_PINS_I2C;
  //bring UCB0 out of reset state
  UCB0CTLW0&=~UCSWRST;
  //enable I2C interrupts
  UCB0IE|=UCNACKIE|UCSTTIE|UCSTPIE|UCALIE|UCCLTOIE|UCTXIE0|UCRXIE0|UCTXIE1|UCRXIE1|UCTXIE2|UCRXIE2|UCTXIE3|UCRXIE3;
  //============[setup SPI]============
  //put UCA0 into reset state
  UCA0CTLW0|=UCSWRST;
  //set MSB first, 3 wire SPI mod, 8 bit words, clock off of SMCLK, keep reset
  UCA0CTLW0=UCMSB|UCMODE_0|UCSYNC|UCSSEL__SMCLK|UCSWRST;
  //clock UCA0 off of SMCLK
  //set SPI clock to 3.2MHz
  UCA0BRW=5;
  //set SPI clock to 1MHz
  //UCA0BR0=0x10;
  //UCA0BR1=0;
  //set SPI clock to 250kHz
  //UCA0BR0=0x40;
  //UCA0BR1=0;
  //leave UCA1 in reset state until it is used for communication
  #ifdef CDH_LIB
      //set lines to be pulled down only on CDH
      P3OUT&=~BUS_PINS_SPI;
      //enable pull resistors for SPI pins only on CDH
      P3REN|=BUS_PINS_SPI;
  #endif
    
  //======[setup pin interrupts]=======

  //rising edge
  P2IES=0x00;
  //falling edge
  //P2IES=0xFF;
  
  
  #ifdef CDH_LIB
    //pull down resistors
    P2OUT=0;
    //pull up resistors
    //P2OUT=0xFF;
    //enable pull resistors
    P2REN=0xFF;
  #else
    //disable pullups
    P2REN=0;
  #endif
  
  //clear flags
  P2IFG=0;
  //enable interrupts
  P2IE=0xFF;

   //create a main task with maximum priority so other tasks can be created without interruption
  //this should be called before other tasks are created
  ctl_task_init(&idle_task, 255, "idle");  

  //start timerA
  start_timerA();

}

int BUS_stop_interrupts(void){
    return ctl_global_interrupts_set(0);
}

void BUS_restart_interrupts(int int_stat){
    //check if we should re-enable interrupts
    if (int_stat){
        //setup timer A to clear TAR value 
        init_timerA();
        //start timer A
        start_timerA();
        //kick the watchdog to get it running again
        WDT_KICK();
        //re-enable interrupts
        ctl_global_interrupts_enable();
    }
}
