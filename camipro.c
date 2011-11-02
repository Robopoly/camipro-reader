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
 */

#include <string.h>
#include <c8051f310.h>

#include "camipro.h"
#include "spi.h"

#define BUFFER_SIZE	32

#define GetRegPage(addr) (0x80|(addr>>3))

// Function prototypes
void SYSCLK_Init (void);
void PORT_Init (void);
void SPI0_Init(void);

void writeRegister(unsigned char address, unsigned char value);
unsigned char readRegister(unsigned char address);
void setBitMask(unsigned char reg,unsigned char mask);
void clearBitMask(unsigned char reg,unsigned char mask);
void flushFIFO(void);

void RFIDTransceive(unsigned char size);

// Global buffers
unsigned char spi_input_buffer[BUFFER_SIZE] = {0};
unsigned char spi_output_buffer[BUFFER_SIZE] = {0};



//////////////////////////////////////////////////////////////////////
//			RF - R E S E T 
///////////////////////////////////////////////////////////////////////
void I1PcdRfReset()   		// time periode in milliseconds 
{
	int i;
	clearBitMask(RFID_REG_TX_CTRL,0x03);	//	Tx2RF-En, Tx1RF-En disablen 
	//T0_Wait_ms(ms);

	for(i=0; i<20000; i++);
	setBitMask(RFID_REG_TX_CTRL,0x03);	 	//	Tx2RF-En, Tx1RF-En enable 
	//init_stat_tbl();			        //initialises (clears) timeslot	status table 
    //TimeSlots = 0; 
}

void I1PcdConfig(void) 
{
	writeRegister(RFID_REG_TX_CTRL,0x48);
	writeRegister(RFID_REG_CW_COND,0x3F);
	writeRegister(RFID_REG_MOD_COND,0x02);	// must	be measured for	15% Modulation Index 
	writeRegister(RFID_REG_CODER_CTRL,0x2C);
	writeRegister(RFID_REG_MOD_WIDTH,0x3F); 
	writeRegister(RFID_REG_MOD_WIDTH_SOF,0x3F); 
	writeRegister(RFID_REG_BFRAM,0x00); //RFU17
 
	writeRegister(RFID_REG_RX_CTRL_1,0x8B); 
	writeRegister(RFID_REG_DEC_CTRL,0x00); 
	writeRegister(RFID_REG_BIT_PHASE,0x52);	 
	writeRegister(RFID_REG_RX_THRESH,0x46); //66
	writeRegister(RFID_REG_BPSKD_CTRL,0x00); 
	writeRegister(RFID_REG_RX_CTRL_2,0x01); 
 	writeRegister(RFID_REG_CLK_Q_CTRL,0x00); 
 
	writeRegister(RFID_REG_RX_WAIT,0x08); 
	writeRegister(RFID_REG_CHANNEL_RED,0x0C); 
	writeRegister(RFID_REG_CRC_PS_LSB,0xFE); 
	writeRegister(RFID_REG_CRC_PS_MSB,0xFF); 
	writeRegister(RFID_REG_TIME_SLOT_PER,0x00); 
	writeRegister(RFID_REG_MFOUT_SEL,0x02);   	// enable SIGOUT = envelope 
	writeRegister(RFID_REG_RFU27,0x00);   				// enable SIGOUT = envelope 
 
	// PAGE 5  FIFO, Timer and IRQ-Pin Configuration 
	writeRegister(RFID_REG_FIFO_LEVEL,0x20); 
	writeRegister(RFID_REG_TIMER_CLK,0x0B); 
	writeRegister(RFID_REG_TIMER_CTRL,0x02); // TStopRxEnd=0,TStopRxBeg=1,TStartTxEnd=1,TStartTxBeg=0 
	writeRegister(RFID_REG_TIMER_RELOAD,0x00); 
 
	flushFIFO();		           // empty	FIFO 
 
	writeRegister(RFID_REG_IRQ_PIN_CFG,0x03); //	interrupt active low enable
	I1PcdRfReset();	           // Rf - reset and enable output driver
}

void init_StdMode_15693(void) 
{ 
	writeRegister(RFID_REG_CODER_CTRL,0x2E); 
	writeRegister(RFID_REG_DEC_CTRL,0x34); 
	writeRegister(RFID_REG_RX_WAIT,0x08);			// 256/fc => 0x01 = 18.88 us 0x08 = 151us 
	writeRegister(RFID_REG_CHANNEL_RED,0x2C); 
	writeRegister(RFID_REG_CRC_PS_LSB,0xFF); 
	writeRegister(RFID_REG_CRC_PS_MSB,0xFF); 
	writeRegister(RFID_REG_TIMER_CLK,0x0B); 
	writeRegister(RFID_REG_BIT_PHASE,0xCD); 
	writeRegister(RFID_REG_MOD_WIDTH_SOF,0x3F); 
}


void IcodeInit()
{
	I1PcdConfig();
	init_StdMode_15693();
}

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
void main (void)
{
	unsigned char i, temp;

	PCA0MD &= ~0x40;					// WDTE = 0 (clear watchdog timer enable)

	SYSCLK_Init ();						// Initialize system clock to 24.5MHz
	PORT_Init ();						// Initialize crossbar and GPIO
	SPI0_Init();

	EA = 0;								// disable global interrupts
	LED_R = 1;

	// Initialization steps
	temp = readRegister(RFID_REG_CMD);
	if(temp != RFID_CMD_IDLE) {
		if(temp == RFID_CMD_STARTUP) {
			// c8051f310 is in startup phase, waiting
			while(readRegister(RFID_REG_CMD) != RFID_CMD_IDLE);
		} else {
			// Chip is stuck on any other command, reseting to idle
			writeRegister(RFID_REG_CMD, RFID_CMD_IDLE);
			
			// Empty buffer if necessary
			temp = readRegister(RFID_REG_DATA_LEN);
			for(i=0; i<temp; i++) {
				readRegister(RFID_REG_DATA);
			}
		}
	}

	for(i=0; i<200; i++);

	IcodeInit();

	for(i=0; i<100; i++);

	//From "idle"
	writeRegister(RFID_REG_IR_EN,	0x3f); // disable interrupts 
	writeRegister(RFID_REG_IR_RQ,	0x3f);
	clearBitMask(RFID_REG_DEC_CTRL, 0x40);  //	Rx Multiple	disable 
	writeRegister(RFID_REG_FIFO_LEVEL,0x20); 
	writeRegister(RFID_REG_CMD, RFID_CMD_IDLE);	        // command idle 

	// At this point the chip is supposed to be correctly configured and ready to transmit

	spi_output_buffer[0] = 0x26; //0x24;
	spi_output_buffer[1] = 0x01;
	spi_output_buffer[2] = 0x00;

	for(i=0; i<200; i++);

	RFIDTransceive(3);
	
	LED_R = 0;
	LED_G = 1;

 	while (1);
}


//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// This routine initializes the system clock to use the internal 24.5MHz / 8
// oscillator as its clock source.  Also enables missing clock detector reset.
//
void SYSCLK_Init (void)
{
   OSCICN = 0x83;                         // configure internal oscillator for its highest frequency
   RSTSRC = 0x04;                         // enable missing clock detector
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Configure the Crossbar and GPIO ports.
// P3.3 - LED (push-pull)
//
void PORT_Init (void)
{							// assignments
	XBR0	= 0x02;				// SPI
	XBR1	= 0x40;				// Enable crossbar and weak pull-ups	

	P0MDOUT = 0x0D;				// Make SCK, MOSI, and NSS push-pull (for SPI)
	P2MDOUT	= 0x0F;
	P3MDOUT	|= 0x08;			// enable LED as a push-pull output	
}

void SPI0_Init()
{   
	SPI0CFG   	= 0x40;			// Enable the SPI as a Master                                   // CKPHA = '0', CKPOL = '0'   
	SPI0CN	    = 0x0D;			// 4-wire Single Master, SPI enabled
	SPI0CKR   	= 3;			// (SYSCLK/(2*SPICLK))-1;	// SPI clock frequency equation from the datasheet
	ESPI0 		= 0;			// Disable SPI interrupts   
}


// RFID Routines

/**
 * Transmit an ISO 15693 frame, and read data back
 * Received data is written in spi_input_buffer
 *
 * @param size
 *		number of bytes to write
 */
void RFIDTransceive(unsigned char size)
{
	unsigned char TimerReload = 0x1F; // For inventory read
	//unsigned char status, err;
	unsigned char i, s;

	//writeRegister(RFID_REG_ERROR, 0x00);
	readRegister(RFID_REG_CMD);
	readRegister(RFID_REG_ERROR);

	setBitMask(RFID_REG_CHANNEL_RED, 0x04);	// enable TxCRC 
	writeRegister(RFID_REG_TIMER_RELOAD, TimerReload); 
	writeRegister(RFID_REG_TIMER_CTRL,0x06); 		// TStopRxEnd=0,TStopRxBeg=1, TStartTxEnd=1,TStartTxBeg=0

	flushFIFO();

	writeRegister(RFID_REG_CMD, 0x00); 
	//SPIWrite(RFID_REG_IR_EN, 0x81);		// set LowAlertEn to write data	to FIFO	(with RC500Isr) 
	//SPIWrite(RFID_REG_IR_RQ, 0x3F); 		//	clear all IRqs 
	//SPIWrite(RFID_REG_IR_EN, 0x38 | 0x80);	 //	enable TxIEn, RxIEn, TIen 
	writeRegister(RFID_REG_IR_EN, 0x00);
	writeRegister(RFID_REG_IR_RQ, 0x00);



	writeRegister(RFID_REG_CMD, RFID_CMD_TRANSCEIVE);	// start to send command to label 

	for(i=0; i<size; i++) {
		writeRegister(RFID_REG_DATA, spi_output_buffer[i]);
	}


	//SPIWrite(RFID_REG_CMD, RFID_CMD_TRANSCEIVE);

	do {
		i = readRegister(RFID_REG_STATUS);
		//readRegister(RFID_REG_ERROR);
		if(i&0x04) {
			s = readRegister(RFID_REG_ERROR);
		}
	} while((i & 0x70) != 0);

	i = 0;
	s = readRegister(RFID_REG_DATA_LEN);
	for(i=0; i<s; i++) {
		spi_input_buffer[i] = readRegister(RFID_REG_DATA);
	}
}


/**
 * Generic write
 */
void writeRegister(unsigned char address, unsigned char value)
{
	spi_wr(0x00, GetRegPage(address));	// select appropriate page by writing the page number to Page0 register
	spi_wr(address&0x07, value);		// write value at the specified address
}

/**
 * Generic read
 */
unsigned char readRegister(unsigned char address)
{
	spi_wr(0x00, GetRegPage(address));	// select appropriate page by writing the page number to Page0 register
	return spi_rd(address&0x07);		//read value at the specified register
}

/**
 * Set a bit mask
 */
void setBitMask(unsigned char reg,unsigned char mask)  
{
	char tmp = readRegister(reg);
	writeRegister(reg,tmp|mask);						// set bit mask
}

/**
 * Clear a bit mask
 */
void clearBitMask(unsigned char reg,unsigned char mask) 
{
   char tmp = readRegister(reg);
   writeRegister(reg,tmp & ~mask);  											// clear bit mask
}

/**
 * Flush FIFO buffer
 */
void flushFIFO(void)
{  
   setBitMask(RFID_REG_CONTROL,0x01);
}

