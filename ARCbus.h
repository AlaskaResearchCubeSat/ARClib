#ifndef __ARC_BUS_H
#define __ARC_BUS_H

#include <ctl.h>

//Macros for watchdog interaction
#define WDT_KICK()        (WDTCTL=WDTPW|WDTCNTCL|WDTSSEL)
//#define WDT_KICK          WDT_STOP
#define WDT_STOP()        (WDTCTL=WDTPW|WDTHOLD|WDTCNTCL)

//thread priorities
enum{BUS_PRI_EXTRA_LOW=20,BUS_PRI_LOW=50,BUS_PRI_NORMAL=80,BUS_PRI_HIGH=110,BUS_PRI_EXTRA_HIGH=140,BUS_PRI_EXTREME=170,BUS_PRI_CRITICAL=200};

//priority for main arcbus task
#define BUS_PRI_ARCBUS        (BUS_PRI_EXTRA_HIGH+20)
//priority for arcbus helper task
#define BUS_PRI_ARCBUS_HELPER (BUS_PRI_EXTRA_HIGH+18)


//Flags for events handled by BUS functions (ex BUS_cmd_tx)
enum{BUS_EV_CMD_NACK=(1<<0),BUS_EV_I2C_COMPLETE=(1<<1),BUS_EV_I2C_NACK=(1<<2),BUS_EV_SPI_COMPLETE=(1<<3),BUS_EV_I2C_ABORT=(1<<4),BUS_EV_SPI_NACK=(1<<5)};
//all events for SPI master
#define BUS_EV_SPI_MASTER           (BUS_EV_SPI_COMPLETE)
//all events created by master transactions
#define BUS_EV_I2C_MASTER           (BUS_EV_I2C_COMPLETE|BUS_EV_I2C_NACK|BUS_EV_I2C_ABORT)

//flags for events handled by the subsystem
enum{SUB_EV_PWR_OFF=(1<<0),SUB_EV_PWR_ON=(1<<1),SUB_EV_SEND_STAT=(1<<2),SUB_EV_SPI_DAT=(1<<3),
     SUB_EV_SPI_ERR_CRC=(1<<4),SUB_EV_SPI_ERR_BUSY=(1<<5),SUB_EV_ASYNC_OPEN=(1<<6),SUB_EV_ASYNC_CLOSE=(1<<7),
     SUB_EV_INT_0=(1<< 8),SUB_EV_INT_1=(1<< 9),SUB_EV_INT_2=(1<<10),SUB_EV_INT_3=(1<<11),
     SUB_EV_INT_4=(1<<12),SUB_EV_INT_5=(1<<13),SUB_EV_INT_6=(1<<14),SUB_EV_INT_7=(1<<15)
     };
//shift to apply to interrupt flags
#define SUB_EV_INT_SHIFT        8

//all subsystem events
#define SUB_EV_ALL                  (SUB_EV_PWR_OFF|SUB_EV_PWR_ON|SUB_EV_SEND_STAT|SUB_EV_SPI_DAT|SUB_EV_SPI_ERR_CRC|SUB_EV_INT_0|SUB_EV_INT_1|SUB_EV_INT_2|SUB_EV_INT_3|SUB_EV_INT_4|SUB_EV_INT_5|SUB_EV_INT_6|SUB_EV_INT_7)
//all subsystem events but pin interrupts
#define SUB_EV_NO_INT               (SUB_EV_PWR_OFF|SUB_EV_PWR_ON|SUB_EV_SEND_STAT|SUB_EV_SPI_DAT|SUB_EV_SPI_ERR_CRC)
//only pin interrupts
#define SUB_EV_INT                  (SUB_EV_INT_0|SUB_EV_INT_1|SUB_EV_INT_2|SUB_EV_INT_3|SUB_EV_INT_4|SUB_EV_INT_5|SUB_EV_INT_6|SUB_EV_INT_7)
//only SPI subsystem events
#define SUB_EV_SPI                  (SUB_EV_SPI_DAT|SUB_EV_SPI_ERR_CRC|SUB_EV_SPI_ERR_BUSY)


//command table for ARCBUS commands
enum{CMD_PING=7,CMD_NACK=51,CMD_SPI_COMPLETE,CMD_SPI_RDY,CMD_SUB_ON,CMD_SUB_OFF,CMD_SUB_POWERUP,CMD_RESET,CMD_SUB_STAT,
     CMD_SPI_CLEAR,CMD_EPS_STAT,CMD_LEDL_STAT,CMD_ACDS_STAT,CMD_COMM_STAT,CMD_IMG_STAT,CMD_ASYNC_SETUP,
     CMD_ASYNC_DAT,CMD_SPI_DATA_ACTION,CMD_MAG_DATA,CMD_MAG_SAMPLE_CONFIG,CMD_ERR_REQ,CMD_IMG_READ_PIC,
     CMD_IMG_TAKE_TIMED_PIC,CMD_IMG_TAKE_PIC_NOW,CMD_GS_DATA};

//bit to allow NACK to be sent
#define CMD_TX_NACK                 (0x80)
//mask for address in command
#define CMD_ADDR_MASK               (0x7F)

//length of SPI CRC
#define BUS_SPI_CRC_LEN             (2)
//length of I2C CRC
#define BUS_I2C_CRC_LEN             (1)
//length of I2C packet header
#define BUS_I2C_HDR_LEN             (2)

//maximum packet length that can fit in the receive buffer
#define BUS_I2C_MAX_PACKET_LEN      (30)

//Return values from bus functions
enum{RET_SUCCESS=0,ERR_BAD_LEN=-1,ERR_CMD_NACK=-2,ERR_I2C_NACK=-3,ERR_UNKNOWN=-4,ERR_BAD_ADDR=-5,ERR_BAD_CRC=-6,ERR_TIMEOUT=-7,ERR_BUSY=-8,ERR_INVALID_ARGUMENT=-9,ERR_PACKET_TOO_LONG=-10,ERR_I2C_ABORT=-11};

//Return values for SUB_parseCmd these will be send as part of the NACK packet
enum{ERR_PK_LEN=1,ERR_UNKNOWN_CMD=2,ERR_SPI_LEN=3,ERR_BAD_PK=4,ERR_SPI_BUSY=5,ERR_BUFFER_BUSY=6,ERR_ILLEAGLE_COMMAND=7};

//table of board addresses
//BUS_ADDR_GC is general call address which every board will acknowledge for receiving
enum{BUS_ADDR_LEDL=0x11,BUS_ADDR_ACDS=0x12,BUS_ADDR_COMM=0x13,BUS_ADDR_IMG=0x14,BUS_ADDR_CDH=0x15,BUS_ADDR_GC=0};

//data to be sent over I2C when there is no data to transmit
#define BUS_I2C_DUMMY_DATA  (0xFF)

//data to be sent over SPI when there is no data to transmit
#define BUS_SPI_DUMMY_DATA  (0xFF)

//flags for BUS_cmd_tx
enum{BUS_CMD_FL_NACK=0x02};

//Power states
enum{SUB_PWR_OFF=0,SUB_PWR_ON};

//used for BUS_cmd_tx
enum{BUS_I2C_SEND_BGND=1,BUS_I2C_SEND_FOREGROUND=0};

//I2C modes
enum {BUS_I2C_IDLE=0,BUS_I2C_TX=1,BUS_I2C_RX};

//SPI modes
enum{BUS_SPI_IDLE=0,BUS_SPI_SLAVE,BUS_SPI_MASTER};
    
//SPI data actions
enum{SPI_DAT_ACTION_INVALID=0,SPI_DAT_ACTION_SD_WRITE,SPI_DAT_ACTION_NULL,SPI_DAT_ACTION_PRINT};
    
//LP main loop low power modes
enum{ML_LP_EXIT,ML_LPM0,ML_LPM1,ML_LPM2,ML_LPM3,ML_LPM4};

//SPI Data types
enum{SPI_BEACON_DAT,SPI_IMG_DAT,SPI_LEDL_DAT,SPI_ERROR_DAT};
    
//error request types
enum{ERR_REQ_REPLAY=0};
    
//Alarm numbers for BUS alarms
enum{BUS_ALARM_0=0,BUS_ALARM_1,BUS_NUM_ALARMS};

//ticker for time keeping
typedef unsigned long ticker;

//low power mode setting in low power main loop
extern char BUS_lp_mode;

//struct for I2C status
typedef struct{
  struct {
    unsigned char *ptr;
    short len,idx;
  }rx;
  struct {
    const unsigned char *ptr;
    short len,idx;
  }tx;
  unsigned short mode;
  CTL_MUTEX_t mutex;
  short mutex_release;
}BUS_I2C_STAT;

//struct for SPI status
typedef struct{
  unsigned char *tx,*rx;
  unsigned short len;
  unsigned short mode;
}BUS_SPI_STAT;

//struct for BUS status
typedef struct{
  BUS_I2C_STAT i2c_stat;
  BUS_SPI_STAT spi_stat;
  CTL_EVENT_SET_t events;
}BUS_STAT;

//bus status
extern BUS_STAT arcBus_stat;

//events for subsystems
extern CTL_EVENT_SET_t SUB_events;

//keep track of power status
extern unsigned short powerState;

//ARClib version
extern const char ARClib_version[];

//setup clocks and low tasking stuff for ARC
void ARC_setup(void);
//low voltage version
void ARC_setup_lv(void);

//setup clocks
void initCLK(void);
//setup clocks for low voltage
void initCLK_lv(void);

//wait for power supply to ramp up then turn on SVS
short SVS_ramp(unsigned short timeout);

//setup the ARC bus
void initARCbus(unsigned char addr);
//setup the ARC bus for the case when all other systems are off
void initARCbus_pd(unsigned char addr);

//Enter the Idle loop. Start the ARCbus tasks and drop idle tasks to lowest priority
void mainLoop(void);
//enter low power main loop. allows the MSP to go into all low power modes
void mainLoop_lp(void);
//main loop testing function, start ARC_Bus task then enter Idle task
void mainLoop_testing(void (*cb)(void));


//send packet over the bus
int BUS_cmd_tx(unsigned char addr,unsigned char *buff,unsigned short len,unsigned short flags,short bgnd);
//Send data over SPI
int BUS_SPI_txrx(unsigned char addr,unsigned char *tx,unsigned char *rx,unsigned short len);
//Setup buffer for command 
unsigned char *BUS_cmd_init(unsigned char *buf,unsigned char id);

//Function provided by subsystem code to parse subsystem commands
int SUB_parseCmd(unsigned char src,unsigned char cmd,unsigned char *dat,unsigned short len);
//get current time
ticker get_ticker_time(void);
//set current time
void set_ticker_time(ticker nt);
//set and get current time
ticker setget_ticker_time(ticker nt);

//get and lock buffer
void* BUS_get_buffer(CTL_TIMEOUT_t t, CTL_TIME_t timeout);
//unlock buffer
void BUS_free_buffer(void);
//get buffer when it was locked by an ARCbus event
void* BUS_get_buffer_from_event(void);
//free buffer that was locked by an ARCbus event
void BUS_free_buffer_from_event(void);
//get the size of the buffer
const unsigned int BUS_get_buffer_size(void);



//check if communicating with a board
int async_isOpen(void);

//Open asynchronous communications with a board
int async_open(unsigned char addr);

//close current connection
int async_close(void);

//transmit a charecter
int async_TxChar(unsigned char c);
int async_Getc(void);
int async_CheckKey(void);
//setup events for byte queue
void async_setup_events(CTL_EVENT_SET_t *e,CTL_EVENT_SET_t txnotfull,CTL_EVENT_SET_t rxnotempty);
//setup closed event
void async_setup_close_event(CTL_EVENT_SET_t *e,CTL_EVENT_SET_t closed);
//send a chunk of async data from the queue
int async_send_data(void);

void reset(unsigned char level,unsigned short source,int err, unsigned short argument);

//get error string for bus errors
const char *BUS_error_str(int error);

//stop global interrupts from happening 
int BUS_stop_interrupts(void);

//gracefully restart global interrupts
void BUS_restart_interrupts(int int_stat);

//set alarm to give an event at the given time
int BUS_set_alarm(unsigned char num,ticker time,CTL_EVENT_SET_t *e,CTL_EVENT_SET_t event);
//get the time an alarm will happen
ticker BUS_get_alarm_time(unsigned char num);
//check if an alarm is free
int BUS_alarm_is_free(unsigned char num);

//free a timer
void BUS_free_alarm(unsigned char num);

#endif
  