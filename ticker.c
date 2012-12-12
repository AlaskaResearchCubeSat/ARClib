#include <ctl.h>
#include <msp430.h>
#include <stdio.h>
#include "ARCbus.h"

//ticker to keep track of time
ticker ticker_time;

//=================[Time ticker functions]=================

//get current ticker time
ticker get_ticker_time(void){
  ticker tmp;
  int en=ctl_global_interrupts_disable();
  tmp=ticker_time;
  if(en){
    ctl_global_interrupts_enable();
  }
  return tmp;
}

//set current ticker time
void set_ticker_time(ticker nt){
  int en=ctl_global_interrupts_disable();
  ticker_time=nt;
  if(en){
    ctl_global_interrupts_enable();
  }
}

//set current ticker time and return old time
ticker setget_ticker_time(ticker nt){
  ticker tmp;
  int en=ctl_global_interrupts_disable();
  tmp=ticker_time;
  ticker_time=nt;
  if(en){
    ctl_global_interrupts_enable();
  }
  return tmp;
}
