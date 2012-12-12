#ifndef __ARC_BUS_H
#define __ARC_BUS_H

#include <ctl.h>

//Macros for watchdog interaction
#define WDT_KICK()        (WDTCTL=WDTPW|WDTCNTCL|WDTSSEL)
//#define WDT_KICK()        (WDTCTL=WDTPW|WDTHOLD|WDTCNTCL)
#define WDT_STOP()        (WDTCTL=WDTPW|WDTHOLD|WDTCNTCL)
#define WDT_RESET()        (WDTCTL=0)

//Flags for events handled by BUS functions (ex BUS_cmd_tx)
enum{BUS_EV_CMD_NACK=(1<<0),BUS_EV_I2C_COMPLETE=(1<<1),BUS_EV_I2C_NACK=(1<<2),BUS_EV_SPI_COMPLETE=(1<<3)};
//all events for SPI master
#define BUS_EV_SPI_MASTER (BUS_EV_SPI_COMPLETE)
//all events created by master transactions
#define BUS_EV_I2C_MASTER (BUS_EV_I2C_COMPLETE|BUS_EV_I2C_NACK)

//flags for events handled by the subsystem
enum{SUB_EV_PWR_OFF=(1<<0),SUB_EV_PWR_ON=(1<<1),SUB_EV_SEND_STAT=(1<<2),SUB_EV_TIME_CHECK=(1<<3),SUB_EV_SPI_DAT=(1<<4),SUB_EV_SPI_ERR_CRC=(1<<5)};
//all subsystem events
#define SUB_EV_ALL        (SUB_EV_PWR_OFF|SUB_EV_PWR_ON|SUB_EV_SEND_STAT|SUB_EV_TIME_CHECK|SUB_EV_SPI_DAT|SUB_EV_SPI_ERR_CRC)

//ERROR codes for I2C functions
enum{I2C_RX_NACK=-1};

//command table for ARCBUS commands
enum{CMD_NACK=51,CMD_SPI_COMPLETE,CMD_SPI_RDY,CMD_SUB_ON,CMD_SUB_OFF,CMD_SUB_POWERUP,CMD_RESET,CMD_SUB_STAT,CMD_SPI_CLEAR};

//bit to allow NACK to be sent
#define CMD_TX_NACK     (0x80)
//mask for address in command
#define CMD_ADDR_MASK   (0x7F)

//length of SPI CRC
#define SPI_CRC_LEN     (2)
//length of I2C CRC
#define I2C_CRC_LEN     (1)
//length of I2C packet header
#define I2C_HDR_LEN     (2)

//Return values from bus functions
enum{RET_SUCCESS=0,ERR_BAD_LEN=-1,ERR_CMD_NACK=-2,ERR_I2C_NACK=-3,ERR_UNKNOWN=-4,ERR_BADD_ADDR=-5,ERR_BAD_CRC=-6,ERR_TIMEOUT=-7};

//Return values for SUB_parseCmd these will be send as part of the NACK packet
enum{ERR_PK_LEN=-1,ERR_UNKNOWN_CMD=-2,ERR_SPI_LEN=-3,ERR_BAD_PK=-4};

//table of board addresses
//BUS_ADDR_GC is general call address which every board will acknowledge for receiving
enum{BUS_ADDR_LEDL=0x11,BUS_ADDR_ACDS=0x12,BUS_ADDR_COMM=0x13,BUS_ADDR_IMG=0x14,BUS_ADDR_CDH=0x15,BUS_ADDR_GC=0};

//data to be sent over I2C when there is no data to transmit
#define I2C_DUMMY_DATA  (0xFF)

//data to be sent over SPI when there is no data to transmit
#define SPI_DUMMY_DATA  (0xFF)

//flags for BUS_cmd_tx
enum{BUS_CMD_FL_NACK=0x02};

//Power states
enum{SUB_PWR_OFF=0,SUB_PWR_ON};

//used for BUS_cmd_tx
enum{SEND_BGND=1,SEND_FOREGROUND=0};

//I2C modes
enum {I2C_IDLE=0,I2C_TX=1,I2C_RX};

//SPI modes
enum{SPI_IDLE=0,SPI_SLAVE,SPI_MASTER};

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
}I2C_STAT;

//struct for SPI status
typedef struct{
  unsigned char *tx,*rx;
  unsigned short len;
  unsigned short mode;
}SPI_STAT;

//struct for BUS status
typedef struct{
  I2C_STAT i2c_stat;
  SPI_STAT spi_stat;
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

#endif
  