/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : SM1.c
**     Project     : BatteryTestSystemC
**     Processor   : MKL25Z128VLK4
**     Component   : SynchroMaster
**     Version     : Component 02.347, Driver 01.01, CPU db: 3.00.000
**     Repository  : Kinetis
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-11-15, 09:37, # CodeGen: 44
**     Abstract    :
**         This component "SynchroMaster" implements MASTER part of synchronous
**         serial master-slave communication.
**     Settings    :
**          Component name                                 : SM1
**          Channel                                        : SPI0
**          Interrupt service/event                        : Enabled
**            Interrupt                                    : 
**            Interrupt from input                         : INT_SPI0
**            Interrupt input priority                     : medium priority
**            Interrupt from output                        : INT_SPI0
**            Interrupt output priority                    : medium priority
**            Input buffer size                            : 0
**            Output buffer size                           : 0
**          Settings                                       : 
**            Width                                        : 8 bits
**            Input pin                                    : Enabled
**              Pin                                        : PTD3/SPI0_MISO/UART2_TX/TPM0_CH3/SPI0_MOSI
**              Pin signal                                 : 
**            Output pin                                   : Enabled
**              Pin                                        : PTD2/SPI0_MOSI/UART2_RX/TPM0_CH2/SPI0_MISO
**              Pin signal                                 : 
**            Clock pin                                    : 
**              Pin                                        : ADC0_SE5b/PTD1/SPI0_SCK/TPM0_CH1
**              Pin signal                                 : 
**            Slave select pin                             : Disabled
**            Clock edge                                   : rising edge
**            Shift clock rate                             : 375 kHz
**            Empty character                              : 0
**            Ignore empty char.                           : no
**            Send MSB first                               : yes
**            Shift clock idle polarity                    : Low
**          Initialization                                 : 
**            Enabled in init. code                        : yes
**            Events enabled in init.                      : yes
**          CPU clock/speed selection                      : 
**            High speed mode                              : This component enabled
**            Low speed mode                               : This component disabled
**            Slow speed mode                              : This component disabled
**          Referenced components                          : 
**            SPIMaster_LDD                                : SPIMaster_LDD
**     Contents    :
**         Enable                - byte SM1_Enable(void);
**         Disable               - byte SM1_Disable(void);
**         RecvChar              - byte SM1_RecvChar(SM1_TComData *Chr);
**         SendChar              - byte SM1_SendChar(SM1_TComData Chr);
**         GetCharsInRxBuf       - word SM1_GetCharsInRxBuf(void);
**         GetCharsInTxBuf       - word SM1_GetCharsInTxBuf(void);
**         SetBaudRateMode       - byte SM1_SetBaudRateMode(byte Mod);
**         SetShiftClockPolarity - byte SM1_SetShiftClockPolarity(byte Edge);
**         SetIdleClockPolarity  - byte SM1_SetIdleClockPolarity(byte Level);
**         GetError              - byte SM1_GetError(SM1_TError *Err);
**
**     Copyright : 1997 - 2015 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file SM1.c
** @version 01.01
** @brief
**         This component "SynchroMaster" implements MASTER part of synchronous
**         serial master-slave communication.
*/         
/*!
**  @addtogroup SM1_module SM1 module documentation
**  @{
*/         

/* MODULE SM1. */

#include "Events.h"
#include "SM1.h"

#ifdef __cplusplus
extern "C" {
#endif 

#define OVERRUN_ERR      0x01U         /* Overrun error flag bit   */
#define TX_BUF_EMPTY     0x02U         /* Tx buffer state flag bit   */
#define CHAR_IN_RX       0x08U         /* Char is in RX buffer     */
#define FULL_TX          0x10U         /* Full transmit buffer     */
#define RUNINT_FROM_TX   0x20U         /* Interrupt is in progress */
#define FULL_RX          0x40U         /* Full receive buffer      */

LDD_TDeviceData *SMasterLdd1_DeviceDataPtr; /* Device data pointer */
static bool EnUser;                    /* Enable/Disable SPI */
static byte SerFlag;                   /* Flags for serial communication */
                                       /* Bits: 0 - OverRun error */
                                       /*       1 - Tx buffer state after init */
                                       /*       2 - Unused */
                                       /*       3 - Char in RX buffer */
                                       /*       4 - Full TX buffer */
                                       /*       5 - Running int from TX */
                                       /*       6 - Full RX buffer */
                                       /*       7 - Unused */
static byte ErrFlag;                   /* Error flags for GetError method */
static SM1_TComData BufferRead;        /* Input char SPI communication */
static SM1_TComData OutBuffer;         /* Output buffer for SPI communication */
static byte SpiClockfeatures;          /* Actual clock features state */
static bool SetAttributeCmd;           /* Set attribute index */
/* Internal method prototypes */
static void HWEnDi(void);

/*
** ===================================================================
**     Method      :  HWEnDi (component SynchroMaster)
**
**     Description :
**         Enables or disables the peripheral(s) associated with the 
**         component. The method is called automatically as a part of the 
**         Enable and Disable methods and several internal methods.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
static void HWEnDi(void)
{
  if (EnUser) {                        /* Enable device? */
    (void)SMasterLdd1_Enable(SMasterLdd1_DeviceDataPtr); /* Enable SPI device */
    if (SetAttributeCmd) {             /* Is request change SPI attribute in enable code? */
      SetAttributeCmd = FALSE;         /* Disable settings SPI attribute in enable code */
      SMasterLdd1_SelectConfiguration(SMasterLdd1_DeviceDataPtr, 0x00U, SpiClockfeatures); /* Set SPI attribute index. */
    }
    (void)SMasterLdd1_ReceiveBlock(SMasterLdd1_DeviceDataPtr, &BufferRead, 1U); /* Receive one data byte */
    if ((SerFlag & FULL_TX) != 0U) {   /* Is any char in transmit buffer? */
      (void)SMasterLdd1_SendBlock(SMasterLdd1_DeviceDataPtr, (LDD_TData *)&OutBuffer, 1U); /* Send one data byte */
    }
  } else {
    (void)SMasterLdd1_Disable(SMasterLdd1_DeviceDataPtr); /* Disable device */
  }
}

/*
** ===================================================================
**     Method      :  SM1_Enable (component SynchroMaster)
**     Description :
**         Enable the component - it starts send and receive functions.
**         Events may be generated ("DisableEvent"/"EnableEvent"). This
**         method cannot be disabled if the Fault mode is enabled. It's
**         intended for re-enabling the module if fault occurred. For
**         more information please see the <"Fault mode" >property .
**     Parameters  : None
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
** ===================================================================
*/
byte SM1_Enable(void)
{
  if (!EnUser) {                       /* Is the device disabled by user? */
    EnUser = TRUE;                     /* If yes then set the flag "device enabled" */
    HWEnDi();                          /* Enable the device */
  }
  return ERR_OK;                       /* OK */
}

/*
** ===================================================================
**     Method      :  SM1_Disable (component SynchroMaster)
**     Description :
**         Disable the component - it stops the send and receive
**         functions. No events will be generated. Note: When this
**         method is called while a transmission is in progress, the
**         data being transmitted/received may be lost.
**     Parameters  : None
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
** ===================================================================
*/
byte SM1_Disable(void)
{
  if (EnUser) {                        /* Is the device enabled by user? */
    EnUser = FALSE;                    /* If yes then set the flag "device disabled" */
    HWEnDi();                          /* Disable the device */
  }
  return ERR_OK;                       /* OK */
}

/*
** ===================================================================
**     Method      :  SM1_RecvChar (component SynchroMaster)
**     Description :
**         If any data is received, this method returns one character,
**         otherwise it returns an error code (it does not wait for
**         data). 
**         For information about SW overrun behavior please see
**         <General info page>.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * Chr             - A pointer to the received character
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK - The valid data is received.
**                           ERR_SPEED - This device does not work in
**                           the active speed mode.
**                           ERR_RXEMPTY - No data in receiver.
**                           ERR_OVERRUN - Overrun error was detected
**                           from the last char or block received. In
**                           polling mode, this error code is returned
**                           only when the hardware supports detection
**                           of the overrun error. If interrupt service
**                           is enabled, and input buffer allocated by
**                           the component is full, the component
**                           behaviour depends on <Input buffer size>
**                           property : if property is 0, last received
**                           data-word is preserved (and previous is
**                           overwritten), if property is greater than 0,
**                           new received data-word are ignored.
**                           ERR_FAULT - Fault error was detected from
**                           the last char or block received. In the
**                           polling mode the ERR_FAULT is return until
**                           the user clear the fault flag bit, but in
**                           the interrupt mode ERR_FAULT is returned
**                           only once after the fault error occured.
**                           This error is supported only on the CPUs
**                           supports the faul mode function - where
**                           <Fault mode> property is available.
** ===================================================================
*/
byte SM1_RecvChar(SM1_TComData *Chr)
{
  register byte FlagTmp;

  if ((SerFlag & CHAR_IN_RX) == 0U) {  /* Is any char in RX buffer? */
    return ERR_RXEMPTY;                /* If no then error */
  }
  EnterCritical();                     /* Disable global interrupts */
  *Chr = BufferRead;                   /* Read the char */
  FlagTmp = SerFlag;                   /* Safe the flags */
  SerFlag &= (byte)~(OVERRUN_ERR | CHAR_IN_RX | FULL_RX); /* Clear flag "char in RX buffer" */
  ExitCritical();                      /* Enable global interrupts */
  if ((FlagTmp & OVERRUN_ERR) != 0U) { /* Is the overrun occurred? */
    return ERR_OVERRUN;                /* If yes then return error */
  } else {
    return ERR_OK;
  }
}

/*
** ===================================================================
**     Method      :  SM1_SendChar (component SynchroMaster)
**     Description :
**         Sends one character to the channel.
**     Parameters  :
**         NAME            - DESCRIPTION
**         Chr             - Character to send
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
**                           ERR_DISABLED - Device is disabled (only if
**                           output DMA is supported and enabled)
**                           ERR_TXFULL - Transmitter is full
** ===================================================================
*/
byte SM1_SendChar(SM1_TComData Chr)
{
  if ((SerFlag & FULL_TX) != 0U) {     /* Is any char in the TX buffer? */
    return ERR_TXFULL;                 /* If yes then error */
  }
  EnterCritical();                     /* Disable global interrupts */
  SerFlag |= FULL_TX;                  /* Set the flag "full TX buffer" */
  OutBuffer = Chr;
  (void)SMasterLdd1_SendBlock(SMasterLdd1_DeviceDataPtr, (LDD_TData *)&OutBuffer, 1U); /* Send one data byte */
  ExitCritical();                      /* Enable global interrupts */
  return ERR_OK;
}

/*
** ===================================================================
**     Method      :  SM1_GetCharsInRxBuf (component SynchroMaster)
**     Description :
**         Returns the number of characters in the input buffer.
**         Note: If the Interrupt service is disabled, and the Ignore
**         empty character is set to yes, and a character has been
**         received, then this method returns 1 although it was an
**         empty character.
**     Parameters  : None
**     Returns     :
**         ---             - Number of characters in the input buffer.
** ===================================================================
*/
word SM1_GetCharsInRxBuf(void)
{
  return ((word)(((SerFlag & CHAR_IN_RX) != 0U)? 1U:0U)); /* Return number of chars in receive buffer */
}

/*
** ===================================================================
**     Method      :  SM1_GetCharsInTxBuf (component SynchroMaster)
**     Description :
**         Returns the number of characters in the output buffer.
**     Parameters  : None
**     Returns     :
**         ---             - Number of characters in the output buffer.
** ===================================================================
*/
word SM1_GetCharsInTxBuf(void)
{
  return (((SerFlag & FULL_TX) != 0U)? 1U : 0U); /* Return number of chars in the transmit buffer */
}

/*
** ===================================================================
**     Method      :  SM1_SetBaudRateMode (component SynchroMaster)
**     Description :
**         This method changes the channel communication speed (baud
**         rate). This method can be used only if you specify a list
**         of possible period settings at design time (see <Timing
**         dialog box> - Runtime setting - from a list of values).
**         Each of these settings constitutes a mode and Processor
**         Expert^[TM] assigns them a mode identifier. The prescaler
**         and compare values corresponding to each mode are
**         calculated at design time. You may switch modes at
**         runtime by referring only to a mode identifier. No
**         run-time calculations are performed, all the calculations
**         are performed at design time.
**     Parameters  :
**         NAME            - DESCRIPTION
**         Mod             - Timing mode to set
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
** ===================================================================
*/
byte SM1_SetBaudRateMode(byte Mod)
{
  LDD_TError Error;

  if (Mod >= 0x02U) {
    return ERR_VALUE;
  }
  SpiClockfeatures &= (byte)0x03U;     /* Calculate BaudRate index */
  SpiClockfeatures |= (byte)(Mod << 2);
  if (EnUser) {
    Error = SMasterLdd1_SelectConfiguration(SMasterLdd1_DeviceDataPtr, 0x00U, SpiClockfeatures); /* Set attribute index. */
  } else {
    SetAttributeCmd = TRUE;            /* Enable setting SPI attribute in enable code */
    Error = ERR_OK;
  }
  return Error;
}

/*
** ===================================================================
**     Method      :  SM1_SetShiftClockPolarity (component SynchroMaster)
**     Description :
**         Sets the shift clock polarity at runtime. Output data will
**         be shifted on the selected edge polarity. The method will
**         disable communication (if enabled), change the shift clock
**         polarity end re-enable the communication (if it was enabled
**         before).
**     Parameters  :
**         NAME            - DESCRIPTION
**         Edge            - Edge polarity.
**                           0-falling edge
**                           1-rising edge
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
**                           ERR_RANGE - Parameter out of range
**                           
**                           [ Version specific information neither for
**                           Freescale HC08 derivatives ] 
**                           [ERR_DISABLED] - Obsolete, this error code
**                           is not used.
** ===================================================================
*/
byte SM1_SetShiftClockPolarity(byte Edge)
{
  if (Edge > 1U) {
    return ERR_RANGE;
  }
  if (Edge != 0U) {
    SpiClockfeatures &= 0xFDU;         /* Set rising edge polarity */
  } else {
    SpiClockfeatures |= 0x02U;         /* Set falling edge polarity */
  }
  EnterCritical();                     /* Disable global interrupts */
  (void)SMasterLdd1_Disable(SMasterLdd1_DeviceDataPtr); /* Disable device */
  (void)SMasterLdd1_Enable(SMasterLdd1_DeviceDataPtr); /* Enable device and cancel receive data request */
  (void)SMasterLdd1_SelectConfiguration(SMasterLdd1_DeviceDataPtr, 0x00U, SpiClockfeatures);
  HWEnDi();                            /* Enable/Disable device */
  ExitCritical();                      /* Enable global interrupts */
  return ERR_OK;
}

/*
** ===================================================================
**     Method      :  SM1_SetIdleClockPolarity (component SynchroMaster)
**     Description :
**         Sets the idle clock polarity at runtime. If the
**         communication does not run, the clock signal will have
**         required level. The method will disable communication (if
**         enabled), change the idle clock polarity end re-enable the
**         communication (if it was enabled before).
**     Parameters  :
**         NAME            - DESCRIPTION
**         Level           - Idle clock polarity:
**                           0-low
**                           1-high
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
**                           ERR_RANGE - Parameter out of range
**                           
**                           [ Version specific information neither for
**                           Freescale HC08 derivatives ] 
**                           [ERR_DISABLED] - Obsolete, this error code
**                           is not used.
**                           
** ===================================================================
*/
byte SM1_SetIdleClockPolarity(byte Level)
{
  if (Level > 1U) {
    return ERR_RANGE;
  }
  SpiClockfeatures = ((SpiClockfeatures & 0xFEU) | Level);
  EnterCritical();                     /* Disable global interrupts */
  (void)SMasterLdd1_Disable(SMasterLdd1_DeviceDataPtr); /* Disable device */
  (void)SMasterLdd1_Enable(SMasterLdd1_DeviceDataPtr); /* Enable device and cancel receive data request */
  (void)SMasterLdd1_SelectConfiguration(SMasterLdd1_DeviceDataPtr, 0x00U, SpiClockfeatures);
  HWEnDi();                            /* Enable/Disable device */
  ExitCritical();                      /* Enable global interrupts */
  return ERR_OK;
}

/*
** ===================================================================
**     Method      :  SM1_GetError (component SynchroMaster)
**     Description :
**         Returns a set of errors on the channel (errors that cannot
**         be returned in given methods). The component accumulates
**         errors in a set; after calling [GetError] this set is
**         returned and cleared. This method is available only if the
**         "Interrupt service/event" property is enabled.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * Err             - A pointer to the returned set of errors
**     Returns     :
**         ---             - Error code (if GetError did not succeed),
**                           possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
** ===================================================================
*/
byte SM1_GetError(SM1_TError *Err)
{
  EnterCritical();                     /* Disable global interrupts */
  Err->err = 0U;
  Err->errName.OverRun = (((ErrFlag & OVERRUN_ERR) != 0U)? 1U : 0U); /* Overrun error */
  ErrFlag = 0x00U;                     /* Reset error flags */
  ExitCritical();                      /* Enable global interrupts */
  return ERR_OK;                       /* OK */
}

/*
** ===================================================================
**     Method      :  SM1_Init (component SynchroMaster)
**
**     Description :
**         Initializes the associated peripheral(s) and the component 
**         internal variables. The method is called automatically as a 
**         part of the application initialization code.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void SM1_Init(void)
{
  SerFlag = 0U;                        /* Reset all flags */
  ErrFlag = 0U;                        /* Reset all flags in mirror */
  SpiClockfeatures = 0x00U;            /* Initialize actual clock feature state */
  SetAttributeCmd = FALSE;             /* Disable settings SPI attribute in enable */
  EnUser = TRUE;                       /* Enable device */
  SMasterLdd1_DeviceDataPtr = SMasterLdd1_Init(NULL); /* Calling init method of the inherited component */
  HWEnDi();                            /* Enable/disable device according to the status flags */
}

#define ON_ERROR        0x01U
#define ON_FULL_RX      0x02U
#define ON_RX_CHAR      0x04U
#define ON_RX_CHAR_EXT  0x08U
/*
** ===================================================================
**     Method      :  SM1_SMasterLdd1_OnBlockReceived (component SynchroMaster)
**
**     Description :
**         This event is called when the requested number of data is 
**         moved to the input buffer. This method is available only if 
**         the ReceiveBlock method is enabled. The event services the 
**         event of the inherited component and eventually invokes other 
**         events.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void SMasterLdd1_OnBlockReceived(LDD_TUserData *UserDataPtr)
{
  register byte Flags = 0U;            /* Temporary variable for flags */

  (void)UserDataPtr;                   /* Parameter is not used, suppress unused argument warning */
  if ((SerFlag & CHAR_IN_RX) != 0U) {  /* Is the overrun error flag set? */
    SerFlag |= OVERRUN_ERR;            /* If yes then set the Error flag for RecvChar/Block method */
    ErrFlag |= OVERRUN_ERR;            /* If yes then set the Error flag for GetError method */
    Flags |= ON_ERROR;                 /* If yes then set the OnError flag */
  }
  SerFlag |= CHAR_IN_RX;               /* Set flag "char in RX buffer" */
  if ((Flags & ON_ERROR) != 0U) {      /* Is any error flag set? */
    SM1_OnError();                     /* Invoke user event */
  } else {
    SM1_OnRxChar();                    /* Invoke user event */
  }
  (void)SMasterLdd1_ReceiveBlock(SMasterLdd1_DeviceDataPtr, &BufferRead, 1U); /* Receive one data byte */
}

#define ON_FREE_TX  0x01U
#define ON_TX_CHAR  0x02U
/*
** ===================================================================
**     Method      :  SM1_SMasterLdd1_OnBlockSent (component SynchroMaster)
**
**     Description :
**         This event is called after the last character from the output 
**         buffer is moved to the transmitter. This event is available 
**         only if the SendBlock method is enabled. The event services 
**         the event of the inherited component and eventually invokes 
**         other events.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void SMasterLdd1_OnBlockSent(LDD_TUserData *UserDataPtr)
{
  (void)UserDataPtr;                   /* Parameter is not used, suppress unused argument warning */
  SerFlag &= (byte)~(FULL_TX);         /* Reset flag "full TX buffer" */
  SM1_OnTxChar();                      /* Invoke user event */
}

/* END SM1. */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

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
