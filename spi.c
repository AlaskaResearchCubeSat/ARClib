#include <msp430.h>
#include "ARCbus.h"
#include "ARCbus_internal.h"

//==============[SPI mode switching commands]==============

//setup UCA0 for master operation
void SPI_master_setup(void){
  //set mode
  arcBus_stat.spi_stat.mode=BUS_SPI_MASTER;
  //put UCA0 into master mode
  UCA0CTL0|=UCMST;
  #ifdef CDH_LIB
      //disable pull resistors for SPI pins only on CDH
      P3REN&=~(BUS_PINS_SPI);
  #endif
  //set pins for SPI usage
  P3SEL0|=BUS_PINS_SPI;
  //bring UCA0 out of reset state
  UCA0CTL1&=~UCSWRST;
}

//setup UCA0 for slave operation
void SPI_slave_setup(void){
  //set mode
  arcBus_stat.spi_stat.mode=BUS_SPI_SLAVE;
  //put UCA0 into slave mode
  UCA0CTL0&=~UCMST;
  #ifdef CDH_LIB
      //disable pull resistors for SPI pins only on CDH
      P3REN&=~(BUS_PINS_SPI);
  #endif
  //set pins for SPI usage
  P3SEL0|=BUS_PINS_SPI;
  //bring UCA0 out of reset state
  UCA0CTL1&=~UCSWRST;
}

//put UCA0 into reset state
void SPI_deactivate(void){
  //put UCA0 into reset state
  UCA0CTL1|=UCSWRST;
  //set pins as inputs
  P3SEL0&=~(BUS_PINS_SPI);
  #ifdef CDH_LIB
      //enable pull resistors for SPI pins only on CDH
      P3REN|=BUS_PINS_SPI;
  #endif
  //set mode
  arcBus_stat.spi_stat.mode=BUS_SPI_IDLE;
}
