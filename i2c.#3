//////////////////////////////////////////////////////////////////////////////
///  _   _        _                            _                             
/// | | | |      |_|                          | |                             
/// | |_| |_____  _ _____           _     _ __| |                             
/// |  _  | ___ || |  _  \  _____  \ \  / // _  |                             
/// | | | | ____|| | |_| | (_____)  \ \/ /( (_| |                             
/// |_| |_|_____)|_|___  |           \__/  \____|                             
///                  __| | Haute Ecole d'Ingénieurie                          
///                 |___/  et de Gestion du canton de Vaud                    
///                                                                           
/// @title    Support du bus I2C pour microcontrolleur 8 bits Silabs          
/// @context  Robotique 2007                                                  
/// @author   Y. Chevallier <nowox@kalios.ch>
///      	  P. Gerber <patrick.gerber@heig-vd.ch>                                  
/// @file     i2c.c                                                           
/// @lang     ASCII/C51                                                       
/// @svn      $Id: i2c.c 165 2007-03-28 12:39:08Z pgerber@heig-vd.ch $                  
//////////////////////////////////////////////////////////////////////////////

#ifndef __C300__ 
#define __C300__ 
	#include <C8051F300.H>
	#include "types.h" 
#endif


//////////////////////////////////////////////////////////////////////////////
/// Définission des constantes globales                                       
//////////////////////////////////////////////////////////////////////////////
#define I2C_WRITE 0x00           // WRITE direction bit
#define I2C_READ  0x01           // READ direction bit
sfr16 TMR2RL = 0xca;                  // Timer2 reload registers
sfr16 TMR2   = 0xcc;                  // Timer2 counter registers

//////////////////////////////////////////////////////////////////////////////
/// Configuration du bus I2C                                                  
//////////////////////////////////////////////////////////////////////////////
#define SYSCLK               24500000  	//!< System clock frequency         
#define I2C_BROADCAST 			 0x00   //!< Addresse du mode broadcast     
#define I2C_ENABLE                  1	
#define I2C_SLAVE_INHIBIT           1
#define I2C_EXTHOLD                 0
#define I2C_SCL_TIMEOUT_DETECTION   1
#define I2C_FREE_TIMEOUT_DETECTION  1
#define I2C_CLKSOURCE_SELECTION     I2C_TIMER1_OVERFLOW
#define I2C_SCL_TIMEOUT_VALUE      25 	//!< in [ms]                           
#define I2C_KBPS               400000 	//!< i2c bitrate in Kbits per second                 
#define I2C_CALLBACK_ELEMENTS       4 	//!< Maximum of callback event can be defined
#define I2C_MAX_BUFFER_LENGTH       4 	//!< Maximum size of i2c messages
#define I2C_NUMBER_OF_TRY           3 	//!< Number of trys for sending a message    
#define I2C_ALL_ADDRESSES         255 

//////////////////////////////////////////////////////////////////////////////
/// Définission des différents états du bus I2C                               
/// MT = Master Transmitter                                                   
/// MR = Master Receiver                                                      
/// ST = Slave Transmitter                                                    
/// SR = Slave Receiver                                                       
//////////////////////////////////////////////////////////////////////////////
#define I2C_MTSTART 0xE0 //!< A master START was generated                   
#define I2C_MTDB    0xC0 //!< A master Data or Address byte was transmitted  
#define I2C_MRDB    0x80 //!< A master data byte was received; ACK requested 

#define I2C_SRADD   0x20 //!< Slave Transmitter data byte transmitted        
#define I2C_SRSTO   0x10 //!< Lost arbitration while attempting à STOP       
#define I2C_SRDB    0x00 //!< A slave byte was received or Lost arbitration  
#define I2C_STDB    0x40 //!< A STOP was detected while an addressed ST    
#define I2C_STCP    0x41 //!< Transfer complete  
#define I2C_STSTO   0x50 //!< A slave address was received   

//////////////////////////////////////////////////////////////////////////////
/// Variables globales                                                        
//////////////////////////////////////////////////////////////////////////////
 
//addresse de ce péripherique                          
uint8 i2c_address;							

// Virtual eeprom 
uint8 * ve_start;
uint8 ve_size;	
uint8 ve_offset;

// Callback
uint8 callback_mode; 
void (*callback)(uint8 reg, uint8 *buffer);

// Buffer
uint8 databuf[8] = 0; 
uint8 datalength = 0; 

//////////////////////////////////////////////////////////////////////////////
/// Initialisation du bus I2C/SMBUS                               
//////////////////////////////////////////////////////////////////////////////
void i2c_init(uint8 address,void * ptr, uint8 size)
{
	//Configuration de la virtual eeprom		
	ve_start = ptr;			//copie du pointeur sur le début de la zone mémoire
	ve_size = size;			//copie la taille
	i2c_address = address<<1;	//copie l'adresse de ce périphérique

	//Initialisation du timer 2	
	TMR2CN = 0x00;
  	CKCON &= ~0x20;
  	TMR2RL = -(I2C_SCL_TIMEOUT_VALUE*SYSCLK/12/1000);
  	TMR2 = TMR2RL;
  	TR2 = 1;

	// Initialisation du SMBUS                                                   
	XBR1 |= 0x04;
	XBR2 |= 0x40;
	SMB0CF = 0x81;
	EIP1    = 0x01;
	EIE1   |= 0x01; 
}

//////////////////////////////////////////////////////////////////////////////
/// Mode Callback                                                 
//////////////////////////////////////////////////////////////////////////////
void i2c_set_callback_mode(void (*function)(uint8 reg, uint8 *buffer))
{
	callback_mode = 1;
	callback = function;
}

//////////////////////////////////////////////////////////////////////////////
/// Détection d'erreur, routine d'interruption                                
//////////////////////////////////////////////////////////////////////////////
void timer_scl_low_timeout_isr(void) interrupt 5
{
  SMB0CF &= ~0x80;    //!< Stop i2c Stack
  SMB0CF |=  0x80;    //!< Restard i2c Stack
  TF2H = 0;           //!< clear interrupt flag
}

//////////////////////////////////////////////////////////////////////////////
/// Routine d'interruption I2C.                                               
/// Traitement des différents états                                           
//////////////////////////////////////////////////////////////////////////////
void i2c_isr(void) interrupt 6 // 7 for C8051F330
{ 
  boolean rxtx;
  static uint8 cpt;


  switch (SMB0CN & 0xF0) //!< Status code for the I2C interface
  {     
    /**
     * Slave Receiver: Start+Address received
     **/
    case I2C_SRADD:

      if(ACKRQ)
      {
        if(((SMB0DAT&0xFE) == i2c_address) || ((SMB0DAT&0xFE) == I2C_BROADCAST)) // Write
        {
		  STA = 0;  // Changé en sta 0 
		  cpt = 0;
          ACK = 1; 
		  if(SMB0DAT&0x01==I2C_READ) // Read
		  {
			if(ve_offset <= ve_size)
				SMB0DAT = ve_start[ve_offset++];
		    if(!(SMB0DAT&0x80))
			{
				STO = 1;
				SI = 0;
				//rxtx = 1;
			} else SI = 0;
		  	while(!TXMODE);
			STO = 0;
		  	
		  }
        }
        else 
        {
          ACK = 0; 
        }
      }
      else
      {
        STA = 0; 
      }
      break;

    /**
     * Slave Receiver: Data received
     **/
    case I2C_SRDB:

      if(ARBLOST) 
      {
        STO = 1; 
        ACK = 1; 
      }
      else
      {
		cpt++;
	  	if(cpt == 1)
			ve_offset = SMB0DAT;
 		else
		{
			if(callback_mode == 1)
				databuf[cpt-2] = SMB0DAT;
			else
			{
	        	if(ve_offset <= ve_size) 
					ve_start[ve_offset++] = SMB0DAT;
			}
		}
		ACK = 1; 
      }
      break;

    /** 
     * Slave Receiver: Stop received
     **/
    case I2C_SRSTO:	 
      ACK = 0; 
      STO = 0;       		//!< No action required (Transfer complete)
	  if(callback_mode == 1)
	  {
	  	callback(ve_offset, &databuf);
	  }
      break; 

    /**
     * Slave Transmitter: Data byte transmitted
     **/      
    case I2C_STDB:
      if(!ARBLOST)
      {
        if(ACK)
        { 
          if(ve_offset <= ve_size)
			SMB0DAT = ve_start[ve_offset++]; 
        }
      }
      break; 
     
	/**
     * Slave Transmitter: Transfer Complete
     **/  
	case I2C_STCP:
		break;

	/**
     * Master Transmitter/Receiver: START condition transmitted
     **/
    case I2C_MTSTART:
      break;

    /**
     * Master Transmitter: Data byte transmitted
     **/
    case I2C_MTDB:    
      break; 

    /**
     * Master Receiver: byte received
     **/    
    case I2C_MRDB: 
      break;

    /** 
     * Default: all other cases undefined
     **/
    default: 
      SMB0CF &= ~0x80;
      SMB0CF |=  0x80; 
  }
  SI=0;              //!< Clear interrupt flag

  if(rxtx)
  {
  	while(!TXMODE)
	STO = 0;
	rxtx = 0;
  }
}                    
