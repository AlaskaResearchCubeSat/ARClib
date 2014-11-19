#include <msp430.h>
#include <stdlib.h>
#include <limits.h>
#include <error.h>
#include "ARCbus.h"
#include "ARCbus_internal.h"

typedef struct{
    ticker time;
    CTL_EVENT_SET_t *e;
    CTL_EVENT_SET_t event;
}ALARM_DAT;

#define     ALARM_MAX_UPDATE_DIFF       (5*60*1024)

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
        if(alarms[i].e!=NULL && alarms[i].event!=0 && alarms[i].time==ticker_time){
            //time elapsed, trigger event
            ctl_events_set_clear(alarms[i].e,alarms[i].event,0);
            //free alarm once triggered
            BUS_free_alarm(i);
        }
    }
}

void BUS_alarm_ticker_update(ticker newt,ticker oldt){
    ticker diff;
    int i;
    //check time difference
    if(newt==oldt){
        //nothing to do
        return;
    }else{
        //subtract times to get difference
        diff=newt-oldt;
        if(diff<ALARM_MAX_UPDATE_DIFF){
            //time went forwards
            report_error(ERR_LEV_INFO,BUS_ERR_SRC_ALARMS,ALARMS_FWD_TIME_UPDATE,diff);
            //trigger alarms that were skipped over
            //loop through all alarms
            for(i=0;i<BUS_NUM_ALARMS;i++){
                //check if time was updated over timeout time
                if(alarms[i].e!=NULL && alarms[i].event!=0 && (alarms[i].time-oldt<=diff)){
                    //time elapsed, trigger event
                    ctl_events_set_clear(alarms[i].e,alarms[i].event,0);
                    //free alarm once triggered
                    BUS_free_alarm(i);
                    //Generate debug message 
                    report_error(ERR_LEV_INFO,BUS_ERR_SRC_ALARMS,ALARMS_ADJ_TRIGGER,i);
                }
            } 
        }else{
            //newt<oldt
            diff=oldt-newt;
            //check if times are close
            if(diff>ALARM_MAX_UPDATE_DIFF){
                //check if difference is large
                if(diff>USHRT_MAX){
                    if(newt-oldt<USHRT_MAX){
                        diff=newt-oldt;
                    }else{
                        diff=USHRT_MAX;
                    }
                }
                //time difference is too large, report error
                report_error(ERR_LEV_ERROR,BUS_ERR_SRC_ALARMS,ALARMS_INVALID_TIME_UPDATE,diff);
                //don't trigger alarms
                return;
            }
            //time went backwards 
            report_error(ERR_LEV_INFO,BUS_ERR_SRC_ALARMS,ALARMS_REV_TIME_UPDATE,diff);
            //TODO: trigger alarms?
        }
    }   
}

