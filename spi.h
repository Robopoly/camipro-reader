#ifndef __SPI__H
#define __SPI__H

/**
 * Write data to the Reader using the SPI Interface
 */
void spi_wr(unsigned char wr_addr,unsigned char wr_data);

/**
 * Read data from the Reader using the SPI Interface
 */
unsigned char spi_rd(unsigned char rd_data);

#endif
