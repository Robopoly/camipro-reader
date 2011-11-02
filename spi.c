/**
 * CAMIPRO Reader for CLRC636
 * http://wiki.robopoly.ch/w/Lecteur_camipro
 * 
 * Copyright (c) 2011 Adrien Beraud
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * 
 * SPI communication functions
 * 
 */

#include <c8051f310.h>
#include "spi.h"

/**
 * SPI_SSEL is cleared to choose the reader as a slave for SPI communication
 */
void ssel_l(void)	
	NSSMD0 = 0;
}

/**
 * SPI_SSEL is set high to release the reader as a slave from SPI communication
 */
void ssel_h(void)
{
	NSSMD0 = 1;
}

/**
 * Transmit data on SPI
 */
unsigned char txrx(unsigned char donnee)
{
	volatile unsigned char readout;

	SPIF = 0;
	SPI0DAT = donnee;

	while(!SPIF);

	readout = SPI0DAT;
	SPIF = 0;

	return readout;
}

/**
 * Write data to the Reader using the SPI Interface
 */
void spi_wr(unsigned char wr_addr,unsigned char wr_data)
{
	unsigned char dummy;
	ssel_l();							  	//clears SSEL0 to start SPI communication
	dummy = txrx((wr_addr<<1) & 0x7E);		//transmit the register address that will be writen to
	dummy = txrx(wr_data); 					//transmit the data byte, no specific format for data							   	
	ssel_h();								//sets SSEL0 to stop SPI communication
}


/**
 * Read data from the Reader using the SPI Interface
 */
unsigned char spi_rd(unsigned char rd_data)
{
	unsigned char content;
	ssel_l();
	txrx((rd_data<<1)|0x80);		 				
	content = txrx(0x00); //transmit "00" and receive last data byte
	ssel_h();
	return content;
}
