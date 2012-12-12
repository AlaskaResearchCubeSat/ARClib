#include <msp430.h>
#include "ARCbus.h"

//==============[SPI mode switching commands]==============

//setup UCA0 for master operation
void SPI_master_setup(void){
  //set mode
  arcBus_stat.spi_stat.mode=SPI_MASTER;
  //put UCA0 into master mode
  UCA0CTL0|=UCMST;
  //set pins for SPI usage
  P3SEL|=BIT0|BIT4|BIT5;
  //bring UCA0 out of reset state
  UCA0CTL1&=~UCSWRST;
}

//setup UCA0 for slave operation
void SPI_slave_setup(void){
  //set mode
  arcBus_stat.spi_stat.mode=SPI_SLAVE;
  //put UCA0 into slave mode
  UCA0CTL0&=~UCMST;
  //set pins for SPI usage
  P3SEL|=BIT0|BIT4|BIT5;
  //bring UCA0 out of reset state
  UCA0CTL1&=~UCSWRST;
}

//put UCA0 into reset state
void SPI_deactivate(void){
  //put UCA0 into reset state
  UCA0CTL1|=UCSWRST;
  //set pins as inputs
  P3SEL&=~(BIT0|BIT4|BIT5);
  //set mode
  arcBus_stat.spi_stat.mode=SPI_IDLE;
}
