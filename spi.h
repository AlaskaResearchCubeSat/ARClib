#ifndef _SPI_H
#define _SPI_H

//setup UCA0 for master operation
void SPI_master_setup(void);
//setup UCA0 for slave operation
void SPI_slave_setup(void);
//put UCA0 into reset state
void SPI_deactivate(void);

#endif
