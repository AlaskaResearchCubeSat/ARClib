#ifndef __ARC_BUS_H
#define __ARC_BUS_H

#include <ctl.h>

//Macros for watchdog interaction
#define WDT_KICK()        (WDTCTL=WDTPW|WDTCNTCL|WDTSSEL)
//#define WDT_KICK          WDT_STOP
#define WDT_STOP()        (WDTCTL=WDTPW|WDTHOLD|WDTCNTCL)
#define WDT_RESET()       (WDTCTL=0)

//Flags for events handled by BUS functions (ex BUS_cmd_tx)
enum{BUS_EV_CMD_NACK=(1<<0),BUS_EV_I2C_COMPLETE=(1<<1),BUS_EV_I2C_NACK=(1<<2),BUS_EV_SPI_COMPLETE=(1<<3)};
//all events for SPI master
#define BUS_EV_SPI_MASTER (BUS_EV_SPI_COMPLETE)
//all events created by master transactions
#define BUS_EV_I2C_MASTER (BUS_EV_I2C_COMPLETE|BUS_EV_I2C_NACK)

//flags for events handled by the subsystem
enum{SUB_EV_PWR_OFF=(1<<0),SUB_EV_PWR_ON=(1<<1),SUB_EV_SEND_STAT=(1<<2),SUB_EV_TIME_CHECK=(1<<3),SUB_EV_SPI_DAT=(1<<4),
     SUB_EV_SPI_ERR_CRC=(1<<5),SUB_EV_ASYNC_OPEN=(1<<6),SUB_EV_ASYNC_CLOSE=(1<<7)};
//all subsystem events
#define SUB_EV_ALL        (SUB_EV_PWR_OFF|SUB_EV_PWR_ON|SUB_EV_SEND_STAT|SUB_EV_TIME_CHECK|SUB_EV_SPI_DAT|SUB_EV_SPI_ERR_CRC)

//command table for ARCBUS commands
enum{CMD_NACK=51,CMD_SPI_COMPLETE,CMD_SPI_RDY,CMD_SUB_ON,CMD_SUB_OFF,CMD_SUB_POWERUP,CMD_RESET,CMD_SUB_STAT,
     CMD_SPI_CLEAR,CMD_EPS_STAT,CMD_LEDL_STAT,CMD_ACDS_STAT,CMD_COMM_STAT,CMD_IMG_STAT,CMD_ASYNC_SETUP,CMD_ASYNC_DAT};

//bit to allow NACK to be sent
#define CMD_TX_NACK     (0x80)
//mask for address in command
#define CMD_ADDR_MASK   (0x7F)

//length of SPI CRC
#define BUS_SPI_CRC_LEN     (2)
//length of I2C CRC
#define BUS_I2C_CRC_LEN     (1)
//length of I2C packet header
#define BUS_I2C_HDR_LEN     (2)

//Return values from bus functions
enum{RET_SUCCESS=0,ERR_BAD_LEN=-1,ERR_CMD_NACK=-2,ERR_I2C_NACK=-3,ERR_UNKNOWN=-4,ERR_BAD_ADDR=-5,ERR_BAD_CRC=-6,ERR_TIMEOUT=-7,ERR_BUSY=-8,ERR_INVALID_ARGUMENT=-9,ERR_FLOW_CTL_STOPPED=-10};

//Return values for SUB_parseCmd these will be send as part of the NACK packet
enum{ERR_PK_LEN=-1,ERR_UNKNOWN_CMD=-2,ERR_SPI_LEN=-3,ERR_BAD_PK=-4,ERR_SPI_BUSY=-5,ERR_BUFFER_BUSY=-6};

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

//ticker for time keeping
typedef unsigned long ticker;

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
  CTL_EVENT_SET_t PortEvents;
}BUS_STAT;

//bus status
extern BUS_STAT arcBus_stat;

//events for subsystems
extern CTL_EVENT_SET_t SUB_events;

//keep track of power status
extern unsigned short powerState;

//setup clocks and low tasking stuff for ARC
void ARC_setup(void);

//setup the ARC bus
void initARCbus(unsigned char addr);

//Enter the Idle loop. Start the ARCbus tasks and drop idle tasks to lowest priority
void mainLoop(void);

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

//get error string for bus errors
const char *BUS_error_str(int error);

#endif
  