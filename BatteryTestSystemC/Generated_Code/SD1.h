/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : SD1.h
**     Project     : BatteryTestSystemC
**     Processor   : MKL25Z128VLK4
**     Component   : SD_Card
**     Version     : Component 01.178, Driver 01.00, CPU db: 3.00.000
**     Repository  : mcuoneclipse
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-11-01, 17:23, # CodeGen: 39
**     Abstract    :
**         Implements interface to SD card for FatFs
**     Settings    :
**          Component name                                 : SD1
**          Block size                                     : 512
**          Cmd wait counter                               : 10
**          Wait Ready Timeout (ms)                        : 500
**          Wait Cmd Timeout (ms)                          : 100
**          Receive Block Timeout (ms)                     : 500
**          SPI Block Transfer                             : yes
**          Hardware                                       : 
**            SW SPI                                       : Disabled
**            HW SPI                                       : Enabled
**              Slow Baud Rate Mode                        : 0
**              Fast Baud Rate Mode                        : 1
**              LDD HW SPI                                 : Disabled
**              non-LDD HW SPI                             : Enabled
**                HW SPI                                   : SM1
**            SPI Read/Write Macros                        : Disabled
**            Slave Select                                 : Enabled
**              LDD SS                                     : Enabled
**                Slave Select Pin                         : LDDSS
**              non-LDD SS                                 : Disabled
**            Activate                                     : Disabled
**            Card detection                               : Disabled
**            Report 'Card present' if no Card detection pin: yes
**            Write protection                             : Disabled
**          System                                         : 
**            Wait                                         : WAIT1
**            Timeout                                      : TMOUT1
**            RTOS                                         : Enabled
**              RTOS                                       : FRTOS1
**     Contents    :
**         Init             - byte SD1_Init(void* unused);
**         Deinit           - byte SD1_Deinit(void* unused);
**         Activate         - void SD1_Activate(void);
**         Deactivate       - void SD1_Deactivate(void);
**         isWriteProtected - bool SD1_isWriteProtected(void);
**         CardPresent      - bool SD1_CardPresent(void);
**         WaitReady        - byte SD1_WaitReady(void);
**         ReceiveDataBlock - bool SD1_ReceiveDataBlock(byte *data, word nofBytes);
**         SendDataBlock    - bool SD1_SendDataBlock(byte *data, byte token, word nofBytes);
**         SendCmd          - byte SD1_SendCmd(byte cmd, dword arg);
**         SetSlowMode      - void SD1_SetSlowMode(void);
**         SetFastMode      - void SD1_SetFastMode(void);
**         InitCommChannel  - void SD1_InitCommChannel(void);
**
**     License   :  Open Source (LGPL)
**     Copyright : (c) Copyright Erich Styger, 2012-2015, all rights reserved.
**     Web       : www.mcuoneclipse.com
**     This an open source software implementing an SD card low level driver useful for the the ChaN FatFS, using Processor Expert.
**     This is a free software and is opened for education,  research and commercial developments under license policy of following terms:
**     * This is a free software and there is NO WARRANTY.
**     * No restriction on use. You can use, modify and redistribute it for personal, non-profit or commercial product UNDER YOUR RESPONSIBILITY.
**     * Redistributions of source code must retain the above copyright notice.
** ###################################################################*/
/*!
** @file SD1.h
** @version 01.00
** @brief
**         Implements interface to SD card for FatFs
*/         
/*!
**  @addtogroup SD1_module SD1 module documentation
**  @{
*/         

#ifndef __SD1_H
#define __SD1_H

/* MODULE SD1. */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited beans */
#include "SM1.h"
#include "SS1.h"
#include "WAIT1.h"
#include "TMOUT1.h"
#include "FRTOS1.h"
/* interface for FatFS low level disk functions */
#include "diskio.h"

DSTATUS SD1_disk_initialize (
        uint8_t drv                     /* Physical drive number (0..) */
);
DSTATUS SD1_disk_status (
        uint8_t drv                     /* Physical drive number (0..) */
);
DRESULT SD1_disk_read (
        uint8_t drv,                    /* Physical drive number (0..) */
        uint8_t *buff,                  /* Data buffer to store read data */
        uint32_t sector,                /* Sector address (LBA) */
        unsigned int count              /* Number of sectors to read (1..255) */
);
#if _READONLY == 0
DRESULT SD1_disk_write (
        uint8_t drv,                    /* Physical drive number (0..) */
        const uint8_t *buff,            /* Data to be written */
        uint32_t sector,                /* Sector address (LBA) */
        unsigned int count              /* Number of sectors to write (1..255) */
);
#endif
DRESULT SD1_disk_ioctl (
        uint8_t drv,                    /* Physical drive number (0..) */
        uint8_t ctrl,                   /* Control code */
        void *buff                      /* Buffer to send/receive control data */
);

#include "Cpu.h"


/* User definitions */

/* distinguished modes for bus activation/deactivation */
#define SD1_ACTIVATE_MODE_SLOW   0
#define SD1_ACTIVATE_MODE_FAST   1

#define SD1_BLOCK_SIZE   512            /* user defined block size */

/******************************* SD Card Standard Commands **********************************/
#define SD1_CMD0  (0x40+0)              /* Resets the SD Memory Card */
#define SD1_CMD1  (0x40+1)              /* Sends host capacity support information and activates the card's
                                           initialization process. HCS is effective when card receives SEND_IF_COND
                                           command. Reserved bits shall be set to '0'. */
#define SD1_CMD6  (0x40+6)              /* Checks switchable function (mode 0) and switches card function (mode 1).*/
#define SD1_CMD8  (0x40+8)              /* Sends SD Memory Card interface condition that includes host supply voltage
                                           information and asks the accessed card whether card can operate in supplied
                                           voltage range. Reserved bits shall be set to '0'.*/
#define SD1_CMD9  (0x40+9)              /* Asks the selected card to send its cardspecific data (CSD)*/
#define SD1_CMD10 (0x40+10)             /* Asks the selected card to send its card identification (CID) */
#define SD1_CMD12 (0x40+12)             /* Forces the card to stop transmission in Multiple Block Read Operation */
#define SD1_CMD13 (0x40+13)             /* Asks the selected card to send its status register. */
#define SD1_CMD16 (0x40+16)             /* Sets a block length (in bytes) for all following block commands (read and
                                           write) of a Standard Capacity Card. Block length of the read and write
                                           commands are fixed to 512 bytes in a High Capacity Card. The length of
                                           LOCK_UNLOCK command is set by this command in both capacity cards.*/
#define SD1_CMD17 (0x40+17)             /* Reads a block of the size selected by the SET_BLOCKLEN command.*/
#define SD1_CMD18 (0x40+18)             /* Continuously transfers data blocks from card to host until interrupted by a
                                           STOP_TRANSMISSION command.*/
#define SD1_CMD24 (0x40+24)             /* Writes a block of the size selected by the SET_BLOCKLEN command. */
#define SD1_CMD25 (0x40+25)             /* Continuously writes blocks of data until �Stop Tran� token is sent
                                          (instead �Start Block�).*/
#define SD1_CMD27 (0x40+27)             /* Programming of the programmable bits of the CSD. */
#define SD1_CMD28 (0x40+28)             /* If the card has write protection features, this command sets the write protection bit
                                           of the addressed group. The properties of write protection are coded in the card
                                           specific data (WP_GRP_SIZE). The High Capacity Card does not support this command.*/
#define SD1_CMD29 (0x40+29)             /* If the card has write protection features, this command clears the write protection
                                           bit of the addressed group. The High Capacity Card does not support this command. */
#define SD1_CMD30 (0x40+30)             /* If the card has write protection features, this command asks the card to send the
                                           status of the write protection bits.6 The High Capacity Card does not support this command. */
#define SD1_CMD32 (0x40+32)             /* Sets the address of the first write block to be erased.*/
#define SD1_CMD33 (0x40+33)             /* Sets the address of the last write block of the continuous range to be erased. */
#define SD1_CMD38 (0x40+38)             /* Erases all previously selected write blocks */
#define SD1_CMD42 (0x40+42)             /* Used to Set/Reset the Password or lock/unlock the card. A transferred data block includes
                                           all the command details - refer to Chapter 4.3.7. The size of the Data Block is defined
                                           with SET_BLOCK_LEN command. Reserved bits in the argument and in Lock Card Data Structure
                                           shall be set to 0. */
#define SD1_CMD55 (0x40+55)             /* Defines to the card that the next command is an application specific command
                                           rather than a standard command */
#define SD1_CMD56 (0x40+56)             /* Used either to transfer a Data Block to the card or to get a Data Block from the card
                                           for general purpose/application specific commands. In case of Standard Capacity SD
                                           Memory Card, the size of the Data Block shall be defined with SET_BLOCK_LEN command.
                                           Block length of this command is fixed to 512-byte in High Capacity Card. */
#define SD1_CMD58 (0x40+58)             /* Reads the OCR register of a card. CCS bit is assigned to OCR[30]. */
#define SD1_CMD59 (0x40+59)             /* Turns the CRC option on or off. A �1� in the CRC option bit will turn the option on,
                                           a �0� will turn it off */
#define SD1_ACMD41 (0xC0+41)            /* SEND_OP_COND (SDC) */
#define SD1_ACMD13 (0xC0+13)            /* SD_STATUS (SDC) */
#define SD1_ACMD23 (0xC0+23)            /* SET_WR_BLK_ERASE_COUNT (SDC) */



byte SD1_Init(void* unused);
/*
** ===================================================================
**     Method      :  SD1_Init (component SD_Card)
**     Description :
**         Initializes the driver
**     Parameters  :
**         NAME            - DESCRIPTION
**       * unused          - unused parameter
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

void SD1_Activate(void);
/*
** ===================================================================
**     Method      :  SD1_Activate (component SD_Card)
**     Description :
**         If multiple devices are used on the same SPI bus, then the
**         device needs to be activated. That way, the different SPI
**         protocol is selected.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void SD1_Deactivate(void);
/*
** ===================================================================
**     Method      :  SD1_Deactivate (component SD_Card)
**     Description :
**         Removes/deactivates the card from the bus
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

#define SD1_isWriteProtected() \
  FALSE                                 /* no hardware to detect write protection, thus none */

/*
** ===================================================================
**     Method      :  SD1_isWriteProtected (component SD_Card)
**     Description :
**         Determines if the card is write protected. Note that this is
**         an indicator only, as it is still possible to write to the
**         card even if the write protection is set on the card!
**     Parameters  : None
**     Returns     :
**         ---             - True if the card has the write protection
**                           set, false otherwise
** ===================================================================
*/

byte SD1_WaitReady(void);
/*
** ===================================================================
**     Method      :  SD1_WaitReady (component SD_Card)
**     Description :
**         Wait until the card is ready
**     Parameters  : None
**     Returns     :
**         ---             - Error code
**                           ERR_OK: device is ready
**                           ERR_BUSY: device is still busy
** ===================================================================
*/

bool SD1_ReceiveDataBlock(byte *data, word nofBytes);
/*
** ===================================================================
**     Method      :  SD1_ReceiveDataBlock (component SD_Card)
**     Description :
**         Retrieve a data block from the device
**     Parameters  :
**         NAME            - DESCRIPTION
**       * data            - Pointer to data buffer
**         nofBytes        - number of bytes to retrieve,
**                           must be a multiple of 4
**     Returns     :
**         ---             - TRUE if reading was going fine, FALSE
**                           otherwise.
** ===================================================================
*/

byte SD1_SendCmd(byte cmd, dword arg);
/*
** ===================================================================
**     Method      :  SD1_SendCmd (component SD_Card)
**     Description :
**         Sends a command to the device and returns the response
**     Parameters  :
**         NAME            - DESCRIPTION
**         cmd             - Command to send
**         arg             - command argument
**     Returns     :
**         ---             - device response
** ===================================================================
*/

byte SD1_ReceiveByte(void);
/*
** ===================================================================
**     Method      :  SD1_ReceiveByte (component SD_Card)
**
**     Description :
**         Receives a byte from the SPI bus
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

#define SD1_CardPresent() \
  TRUE                                  /* no card detection pin, but user wants to report TRUE */

/*
** ===================================================================
**     Method      :  SD1_CardPresent (component SD_Card)
**     Description :
**         Returns true in case a card is present. If there is no card
**         detection pin, then this routine will always return true.
**     Parameters  : None
**     Returns     :
**         ---             - Returns true if card is present, false
**                           otherwise.
** ===================================================================
*/

bool SD1_SendDataBlock(byte *data, byte token, word nofBytes);
/*
** ===================================================================
**     Method      :  SD1_SendDataBlock (component SD_Card)
**     Description :
**         Send a data block to the device
**     Parameters  :
**         NAME            - DESCRIPTION
**       * data            - Pointer to data blocks with 512 bytes
**                           each
**         token           - data/stop token
**         nofBytes        - Number of bytes to send
**     Returns     :
**         ---             - Returns TRUE for success, FALSE for
**                           failure.
** ===================================================================
*/

void SD1_SetFastMode(void);
/*
** ===================================================================
**     Method      :  SD1_SetFastMode (component SD_Card)
**     Description :
**         Switches to fast mode SPI communication speed.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void SD1_InitCommChannel(void);
/*
** ===================================================================
**     Method      :  SD1_InitCommChannel (component SD_Card)
**     Description :
**         Method to initialize the communication channel. This is
**         needed if the bus to the SD card is shared with other
**         devices.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

byte SD1_Deinit(void* unused);
/*
** ===================================================================
**     Method      :  SD1_Deinit (component SD_Card)
**     Description :
**         Driver deinitialization routine.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * unused          - dummy parameter
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

void SD1_SetSlowMode(void);
/*
** ===================================================================
**     Method      :  SD1_SetSlowMode (component SD_Card)
**     Description :
**         Switches to slow mode SPI communication speed.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

/* END SD1. */

#endif
/* ifndef __SD1_H */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
