#include <ctl.h>
#include <msp430.h>
#include "ARCbus.h"

#include "ARCbus_internal.h"

//mutex for buffer locking
CTL_MUTEX_t buffer_mutex;

//buffer for SPI transactions
//static unsigned char Buffer[4096+2];
//static unsigned char Buffer[2048+2];
//static unsigned char Buffer[1024+4];

//setup stuff for buffer usage
void BUS_init_buffer(void){
  //initialize mutex
  ctl_mutex_init(&buffer_mutex);
}

//return buffer size
const unsigned int BUS_get_buffer_size(void){
  //return sizeof(Buffer);
  return 0;
}

//lock buffer and return pointer to buffer
void* BUS_get_buffer(CTL_TIMEOUT_t t, CTL_TIME_t timeout){
  return NULL;

}

//free buffer
void BUS_free_buffer(void){
  ctl_mutex_unlock(&buffer_mutex);
}

//get buffer if it was locked by ARCbus
void* BUS_get_buffer_from_event(void){
  //TODO: error chekcing
  return NULL;
}

//free buffer if it was locket by ARCbus
void BUS_free_buffer_from_event(void){
  //bus internal events
  extern CTL_EVENT_SET_t BUS_INT_events;
  //set event to realese buffer
  ctl_events_set_clear(&BUS_INT_events,BUS_INT_EV_BUFF_UNLOCK,0);
}





