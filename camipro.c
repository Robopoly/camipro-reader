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
#include <c8051f310.h>                    // SFR declarations

#include "camipro.h"


/* SPI globals */
unsigned char spi_input_buffer[MAX_BUFFER_SIZE] = {0};
unsigned char spi_output_buffer[MAX_BUFFER_SIZE] = {0};

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void SYSCLK_Init (void);
void PORT_Init (void);
void SPI0_Init(void);

void RFIDTransceive(unsigned char size);

//Imported from clrc636.c

#define GetRegPage(addr) (0x80 | (addr>>3))

void ssel_l(void)		 													//SPI_SSEL is cleared to choose the reader as a slave
{																			//for SPI communication
	NSSMD0 = 0;
}

void ssel_h(void)	   														//SPI_SSEL is set high to release the reader as a slave
{																			//from SPI communication
	NSSMD0 = 1;
}


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

//write data to the Reader using the SPI Interface, data format: MOSI= adr, data0, data1,........
//all data is written to the same address
//unsigned char spi_wr(unsigned char wr_addr,unsigned char wr_count,unsigned char* wr_data)
void spi_wr(unsigned char wr_addr,unsigned char wr_data)
{
	unsigned char dummy;
	ssel_l();							  	//clears SSEL0 to start SPI communication
	dummy = txrx((wr_addr<<1) & 0x7E);		//transmit the register address that will be writen to
	dummy = txrx(wr_data); 					//transmit the data byte, no specific format for data							   	
	ssel_h();								//sets SSEL0 to stop SPI communication
}

//read data from the Reader using the SPI Interface
//data format is as follows:
//MOSI: addr0, addr1, addr2, ...., datan   , 00
//MISO: XX   , data0, data1, ...., datan-1 ,datan
//unsigned long int spi_rd(unsigned char rd_count,unsigned char* rd_data)
unsigned char spi_rd(unsigned char rd_data)
{
	unsigned char content;
	ssel_l();
	txrx((rd_data<<1)|0x80);		 				
	content = txrx(0x00);				   								//transmit "00" and receive last data byte
	ssel_h();
	return content;
}

//! Write one byte to the reader IC address space
void Write_Register(unsigned char address, unsigned char donnee)
{
	spi_wr(address & 0x07, donnee);
}


//! Read one byte from the reader IC address space
unsigned char Read_Register(unsigned char reg)
{
	return spi_rd(reg & 0x07);
}

///////////////////////////////////////////////////////////////////////////////
//          G E N E R I C    W R I T E
///////////////////////////////////////////////////////////////////////////////
void WriteRC(unsigned char address, unsigned char value)
{
	Write_Register(0x00,GetRegPage(address));	// select appropriate page by writing the page number to Page0 register
	Write_Register(address,value);				// write value at the specified address   
}

///////////////////////////////////////////////////////////////////////////////
//          G E N E R I C    R E A D
///////////////////////////////////////////////////////////////////////////////
unsigned char ReadRC(unsigned char address)
{
	Write_Register(0x00,GetRegPage(address));	// select appropriate page
	return Read_Register(address);				//read value at the specified register, added as modification, 29.05.2007
}


//////////////////////////////////////////////////////////////////////
//   S E T   A   B I T   M A S K 
///////////////////////////////////////////////////////////////////////
void SetBitMask(unsigned char reg,unsigned char mask)  
{
	char tmp = ReadRC(reg);
	WriteRC(reg,tmp|mask);						// set bit mask
}

//////////////////////////////////////////////////////////////////////
//   C L E A R   A   B I T   M A S K 
///////////////////////////////////////////////////////////////////////
void ClearBitMask(unsigned char reg,unsigned char mask) 
{
   char tmp = ReadRC(reg);
   WriteRC(reg,tmp & ~mask);  											// clear bit mask
}

///////////////////////////////////////////////////////////////////////
//                  F L U S H    F I F O
///////////////////////////////////////////////////////////////////////
void FlushFIFO(void)
{  
   SetBitMask(RFID_REG_CONTROL,0x01);
}

////////////////////////////////////////////////////////////////////// 
//			RF - R E S E T 
/////////////////////////////////////////////////////////////////////// 
void I1PcdRfReset()   		// time periode in milliseconds 
{
	int i;
	ClearBitMask(RFID_REG_TX_CTRL,0x03);	//	Tx2RF-En, Tx1RF-En disablen 
	//T0_Wait_ms(ms);

	for(i=0; i<20000; i++);
	SetBitMask(RFID_REG_TX_CTRL,0x03);	 	//	Tx2RF-En, Tx1RF-En enable 
	//init_stat_tbl();			        //initialises (clears) timeslot	status table 
    //TimeSlots = 0; 
}

void I1PcdConfig(void) 
{
	WriteRC(RFID_REG_TX_CTRL,0x48);
	WriteRC(RFID_REG_CW_COND,0x3F);
	WriteRC(RFID_REG_MOD_COND,0x02);	// must	be measured for	15% Modulation Index 
	WriteRC(RFID_REG_CODER_CTRL,0x2C);
	WriteRC(RFID_REG_MOD_WIDTH,0x3F); 
	WriteRC(RFID_REG_MOD_WIDTH_SOF,0x3F); 
	WriteRC(RFID_REG_BFRAM,0x00); //RFU17
 
	WriteRC(RFID_REG_RX_CTRL_1,0x8B); 
	WriteRC(RFID_REG_DEC_CTRL,0x00); 
	WriteRC(RFID_REG_BIT_PHASE,0x52);	 
	WriteRC(RFID_REG_RX_THRESH,0x46); //66
	WriteRC(RFID_REG_BPSKD_CTRL,0x00); 
	WriteRC(RFID_REG_RX_CTRL_2,0x01); 
 	WriteRC(RFID_REG_CLK_Q_CTRL,0x00); 
 
	WriteRC(RFID_REG_RX_WAIT,0x08); 
	WriteRC(RFID_REG_CHANNEL_RED,0x0C); 
	WriteRC(RFID_REG_CRC_PS_LSB,0xFE); 
	WriteRC(RFID_REG_CRC_PS_MSB,0xFF); 
	WriteRC(RFID_REG_TIME_SLOT_PER,0x00); 
	WriteRC(RFID_REG_MFOUT_SEL,0x02);   	// enable SIGOUT = envelope 
	WriteRC(RFID_REG_RFU27,0x00);   				// enable SIGOUT = envelope 
 
	// PAGE 5  FIFO, Timer and IRQ-Pin Configuration 
	WriteRC(RFID_REG_FIFO_LEVEL,0x20); 
	WriteRC(RFID_REG_TIMER_CLK,0x0B); 
	WriteRC(RFID_REG_TIMER_CTRL,0x02); // TStopRxEnd=0,TStopRxBeg=1,TStartTxEnd=1,TStartTxBeg=0 
	WriteRC(RFID_REG_TIMER_RELOAD,0x00); 
 
	FlushFIFO();		           // empty	FIFO 
 
	WriteRC(RFID_REG_IRQ_PIN_CFG,0x03); //	interrupt active low enable
	I1PcdRfReset();	           // Rf - reset and enable output driver
}

void init_StdMode_15693(void) 
{ 
	WriteRC(RFID_REG_CODER_CTRL,0x2E); 
	WriteRC(RFID_REG_DEC_CTRL,0x34); 
	WriteRC(RFID_REG_RX_WAIT,0x08);			// 256/fc => 0x01 = 18.88 us 0x08 = 151us 
	WriteRC(RFID_REG_CHANNEL_RED,0x2C); 
	WriteRC(RFID_REG_CRC_PS_LSB,0xFF); 
	WriteRC(RFID_REG_CRC_PS_MSB,0xFF); 
	WriteRC(RFID_REG_TIMER_CLK,0x0B); 
	WriteRC(RFID_REG_BIT_PHASE,0xCD); 
	WriteRC(RFID_REG_MOD_WIDTH_SOF,0x3F); 
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
	temp = ReadRC(RFID_REG_CMD);
	if(temp != RFID_CMD_IDLE) {
		if(temp == RFID_CMD_STARTUP) {
			// c8051f310 is in startup phase, waiting
			while(ReadRC(RFID_REG_CMD) != RFID_CMD_IDLE);
		} else {
			// Chip is stuck on any other command, reseting to idle
			WriteRC(RFID_REG_CMD, RFID_CMD_IDLE);
			
			// Empty buffer if necessary
			temp = ReadRC(RFID_REG_DATA_LEN);
			for(i=0; i<temp; i++) {
				ReadRC(RFID_REG_DATA);
			}
		}
	}

	for(i=0; i<200; i++);

	IcodeInit();

	for(i=0; i<100; i++);

	//From "idle"
	WriteRC(RFID_REG_IR_EN,	0x3f); // disable interrupts 
	WriteRC(RFID_REG_IR_RQ,	0x3f);
	ClearBitMask(RFID_REG_DEC_CTRL, 0x40);  //	Rx Multiple	disable 
	WriteRC(RFID_REG_FIFO_LEVEL,0x20); 
	WriteRC(RFID_REG_CMD, RFID_CMD_IDLE);	        // command idle 

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

	//WriteRC(RFID_REG_ERROR, 0x00);
	ReadRC(RFID_REG_CMD);
	ReadRC(RFID_REG_ERROR);

	SetBitMask(RFID_REG_CHANNEL_RED, 0x04);	// enable TxCRC 
	WriteRC(RFID_REG_TIMER_RELOAD, TimerReload); 
	WriteRC(RFID_REG_TIMER_CTRL,0x06); 		// TStopRxEnd=0,TStopRxBeg=1, TStartTxEnd=1,TStartTxBeg=0

	FlushFIFO();

	WriteRC(RFID_REG_CMD, 0x00); 
	//SPIWrite(RFID_REG_IR_EN, 0x81);		// set LowAlertEn to write data	to FIFO	(with RC500Isr) 
	//SPIWrite(RFID_REG_IR_RQ, 0x3F); 		//	clear all IRqs 
	//SPIWrite(RFID_REG_IR_EN, 0x38 | 0x80);	 //	enable TxIEn, RxIEn, TIen 
	WriteRC(RFID_REG_IR_EN, 0x00);
	WriteRC(RFID_REG_IR_RQ, 0x00);



	WriteRC(RFID_REG_CMD, RFID_CMD_TRANSCEIVE);	// start to send command to label 

	for(i=0; i<size; i++) {
		WriteRC(RFID_REG_DATA, spi_output_buffer[i]);
	}


	//SPIWrite(RFID_REG_CMD, RFID_CMD_TRANSCEIVE);

	do {
		i = ReadRC(RFID_REG_STATUS);
		//ReadRC(RFID_REG_ERROR);
		if(i&0x04) {
			s = ReadRC(RFID_REG_ERROR);
		}
	} while((i & 0x70) != 0);

	i = 0;
	s = ReadRC(RFID_REG_DATA_LEN);
	for(i=0; i<s; i++) {
		spi_input_buffer[i] = ReadRC(RFID_REG_DATA);
	}
}

