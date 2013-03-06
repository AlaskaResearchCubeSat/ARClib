#include <ctl.h>
#include <msp430.h>
#include "ARCbus.h"

#include "ARCbus_internal.h"

//mutex for buffer locking
CTL_MUTEX_t buffer_mutex;

//buffer for SPI transactions
//TODO: make this buffer available for other uses
static unsigned char Buffer[2048+2];

//setup stuff for buffer usage
void BUS_init_buffer(void){
  //initialize mutex
  ctl_mutex_init(&buffer_mutex);
}

void* BUS_get_buffer(CTL_TIMEOUT_t t, CTL_TIME_t timeout){
  if(ctl_mutex_lock(&buffer_mutex,t,timeout)){
    //lock aquired, return buffer
    return Buffer;
  }else{
    //lock not aquired return NULL
    return NULL;
  }
}

void BUS_free_buffer(void){
  ctl_mutex_unlock(&buffer_mutex);
}

void* BUS_get_buffer_from_event(void){
  //TODO: error chekcing
  return Buffer;
}

void BUS_free_buffer_from_event(void){
  //bus internal events
  extern CTL_EVENT_SET_t BUS_INT_events;
  //set event to realese buffer
  ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_BUFF_UNLOCK,0);
}





