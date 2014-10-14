#include <msp430.h>
#include <stdlib.h>
#include "ARCbus.h"

typedef struct{
    ticker time;
    CTL_EVENT_SET_t *e;
    CTL_EVENT_SET_t event;
}ALARM_DAT;

ALARM_DAT alarms[BUS_NUM_ALARMS];

int BUS_alarm_is_free(unsigned char num){
    if(num>=BUS_NUM_ALARMS){
        return ERR_INVALID_ARGUMENT;
    }
    //check event
    if(alarms[num].e!=NULL || alarms[num].event!=0){
        return ERR_BUSY;
    }
    //timer not in use
    return RET_SUCCESS;
}

int BUS_set_alarm(unsigned char num,ticker time,CTL_EVENT_SET_t *e,CTL_EVENT_SET_t event){
    int ret;
    int en;
    //check if alarm is busy
    ret=BUS_alarm_is_free(num);
    if(ret!=RET_SUCCESS){
        //alarm busy, return error
        return ret;
    }
    en=ctl_global_interrupts_disable();
    //set time
    alarms[num].time=time;
    //set event
    alarms[num].e=e;
    alarms[num].event=event;
    if(en){
        ctl_global_interrupts_enable();
    }
    return RET_SUCCESS;
}

void BUS_free_alarm(unsigned char num){
     if(num>=BUS_NUM_ALARMS){
        return;
    }
    //free timer
    alarms[num].e=0;
    alarms[num].event=NULL;
}

//called from timer ISR
void BUS_timer_timeout_check(void){
    extern ticker ticker_time;
    int i;
    //loop through all inputs
    for(i=0;i<BUS_NUM_ALARMS;i++){
        //check for timeout
        if(alarms[i].e!=NULL || alarms[i].event!=0 && alarms[i].time==ticker_time){
            //time elapsed, trigger event
            ctl_events_set_clear(alarms[i].e,alarms[i].event,0);
        }
    }
}
