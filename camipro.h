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

#ifndef __CAMIPROREADER__H
#define __CAMIPROREADER__H

// 16-bit SFR Definitions for 'F30x
sfr16 TMR2RL	= 0xca;					// Timer2 reload value
sfr16 TMR2		= 0xcc;					// Timer2 counter

// LEDs, buzzer and push-button
sbit LED = P3^3;						// LED='1' means ON
sbit LED_B = P2^0;
sbit LED_G = P2^1;
sbit LED_R = P2^2;
sbit BUZZER = P2^3;

sbit SW2 = P0^7;						// SW2='0' means switch pressed

// Global constants

// Clock frequencies
#define SYSCLK				24500000	// SYSCLK frequency in Hz
#define SPICLK				12845056	// SPI maximum SCK frequency in Hz

// Adress and common values for CLRC632

// Register addresses
#define RFID_REG_PAGE			0x00

// Page 0
#define RFID_REG_CMD			0x01
#define RFID_REG_DATA			0x02
#define RFID_REG_STATUS			0x03
#define RFID_REG_DATA_LEN		0x04
#define RFID_REG_STATUS2		0x05
#define RFID_REG_IR_EN			0x06
#define RFID_REG_IR_RQ			0x07

// Page 1
#define RFID_REG_CONTROL		0x09
#define RFID_REG_ERROR			0x0A
#define RFID_REG_COLL_POS		0x0B

// Page 2
#define RFID_REG_TX_CTRL		0x11
#define RFID_REG_CW_COND		0x12
#define RFID_REG_MOD_COND		0x13
#define RFID_REG_CODER_CTRL		0x14
#define RFID_REG_MOD_WIDTH		0x15
#define RFID_REG_MOD_WIDTH_SOF	0x16
#define RFID_REG_BFRAM			0x17

// Page 3
#define RFID_REG_RX_CTRL_1		0x19
#define RFID_REG_DEC_CTRL		0x1A
#define RFID_REG_BIT_PHASE		0x1B
#define RFID_REG_RX_THRESH		0x1C
#define RFID_REG_BPSKD_CTRL		0x1D
#define RFID_REG_RX_CTRL_2		0x1E
#define RFID_REG_CLK_Q_CTRL		0x1F

// Page 4
#define RFID_REG_RX_WAIT		0x21
#define RFID_REG_CHANNEL_RED	0x22
#define RFID_REG_CRC_PS_LSB		0x23
#define RFID_REG_CRC_PS_MSB		0x24
#define RFID_REG_TIME_SLOT_PER	0x25
#define RFID_REG_MFOUT_SEL		0x26
#define RFID_REG_RFU27			0x27

// Page 5
#define RFID_REG_FIFO_LEVEL		0x29
#define RFID_REG_TIMER_CLK		0x2A
#define RFID_REG_TIMER_CTRL		0x2B
#define RFID_REG_TIMER_RELOAD	0x2C
#define RFID_REG_IRQ_PIN_CFG	0x2D

// Command values (to use with RFID_REG_CMD)
#define RFID_CMD_STARTUP		0x3F
#define RFID_CMD_IDLE			0x00
#define RFID_CMD_TRANSMIT		0x1A
#define RFID_CMD_RECEIVE		0x16
#define RFID_CMD_TRANSCEIVE		0x1E
#define RFID_CMD_READE2			0x03
#define RFID_CMD_WRITEE2		0x01
#define RFID_CMD_LOADCONF		0x07

#endif
