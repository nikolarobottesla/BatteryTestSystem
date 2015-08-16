/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : TmDt1.h
**     Project     : BatteryTestSystemC
**     Processor   : MKL25Z128VLK4
**     Component   : GenericTimeDate
**     Version     : Component 01.020, Driver 01.00, CPU db: 3.00.000
**     Repository  : mcuoneclipse
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-07-26, 01:31, # CodeGen: 5
**     Abstract    :
**         Software date/time module.
**     Settings    :
**          Component name                                 : TmDt1
**          Critical Section                               : CS1
**          Tick Time (ms)                                 : 10
**          RTOS                                           : Enabled
**            RTOS                                         : FRTOS1
**          Shell                                          : Enabled
**            Utility                                      : UTIL1
**            Shell                                        : CLS1
**          Initialization                                 : Enabled
**            Init in startup                              : yes
**            Date                                         : 2015-07-01
**            Time                                         : 12:51:31
**     Contents    :
**         AddTick      - void TmDt1_AddTick(void);
**         AddTicks     - void TmDt1_AddTicks(uint16_t nofTicks);
**         SetTime      - uint8_t TmDt1_SetTime(uint8_t Hour, uint8_t Min, uint8_t Sec, uint8_t Sec100);
**         GetTime      - uint8_t TmDt1_GetTime(TIMEREC *Time);
**         SetDate      - uint8_t TmDt1_SetDate(uint16_t Year, uint8_t Month, uint8_t Day);
**         GetDate      - uint8_t TmDt1_GetDate(DATEREC *Date);
**         ParseCommand - uint8_t TmDt1_ParseCommand(const unsigned char *cmd, bool *handled, const...
**         Init         - void TmDt1_Init(void);
**         DeInit       - void TmDt1_DeInit(void);
**
**     (c) Copyright Freescale Semiconductor, 2014
**     http: www.freescale.com
**     Source code is based on the original TimeDate Processor Expert component.
** ###################################################################*/
/*!
** @file TmDt1.h
** @version 01.00
** @brief
**         Software date/time module.
*/         
/*!
**  @addtogroup TmDt1_module TmDt1 module documentation
**  @{
*/         

#ifndef __TmDt1_H
#define __TmDt1_H

/* MODULE TmDt1. */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited beans */
#include "CS1.h"
#include "FRTOS1.h"
#include "UTIL1.h"
#include "CLS1.h"
#define TmDt1_PARSE_COMMAND_ENABLED  1  /* set to 1 if method ParseCommand() is present, 0 otherwise */

#ifndef __BWUserType_TIMEREC
#define __BWUserType_TIMEREC
  typedef struct {                     /* It contains actual number of hours, minutes, seconds and hundreth of seconds. */
    uint8_t Hour;                      /* hours (0 - 23) */
    uint8_t Min;                       /* minutes (0 - 59) */
    uint8_t Sec;                       /* seconds (0 - 59) */
    uint8_t Sec100;                    /* hundredth of seconds (0 - 99) */
  } TIMEREC;
#endif
#ifndef __BWUserType_DATEREC
#define __BWUserType_DATEREC
  typedef struct {                     /* It contains actual year, month, and day description. */
    uint16_t Year;                     /* years (1998 - 2099) */
    uint8_t Month;                     /* months (1 - 12) */
    uint8_t Day;                       /* days (1 - 31) */
  } DATEREC;
#endif

#include "Cpu.h"




uint8_t TmDt1_SetTime(uint8_t Hour, uint8_t Min, uint8_t Sec, uint8_t Sec100);
/*
** ===================================================================
**     Method      :  TmDt1_SetTime (component GenericTimeDate)
**     Description :
**         This method sets a new actual time.
**     Parameters  :
**         NAME            - DESCRIPTION
**         Hour            - Hours (0 - 23)
**         Min             - Minutes (0 - 59)
**         Sec             - Seconds (0 - 59)
**         Sec100          - Hundredth of seconds (0 - 99)
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

void TmDt1_AddTick(void);
/*
** ===================================================================
**     Method      :  TmDt1_AddTick (component GenericTimeDate)
**     Description :
**         Needs to be called periodically by the application to
**         increase the time tick count.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

uint8_t TmDt1_GetTime(TIMEREC *Time);
/*
** ===================================================================
**     Method      :  TmDt1_GetTime (component GenericTimeDate)
**     Description :
**         This method returns current time.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * Time            - Pointer to the structure TIMEREC. It
**                           contains actual number of hours, minutes,
**                           seconds and hundredth of seconds.
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint8_t TmDt1_SetDate(uint16_t Year, uint8_t Month, uint8_t Day);
/*
** ===================================================================
**     Method      :  TmDt1_SetDate (component GenericTimeDate)
**     Description :
**         This method sets a new actual date.
**     Parameters  :
**         NAME            - DESCRIPTION
**         Year            - Years (16-bit unsigned integer)
**         Month           - Months (8-bit unsigned integer)
**         Day             - Days (8-bit unsigned integer)
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint8_t TmDt1_GetDate(DATEREC *Date);
/*
** ===================================================================
**     Method      :  TmDt1_GetDate (component GenericTimeDate)
**     Description :
**         This method returns current date.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * Date            - Pointer to DATEREC
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

void TmDt1_Init(void);
/*
** ===================================================================
**     Method      :  TmDt1_Init (component GenericTimeDate)
**     Description :
**         Initialization method
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

uint8_t TmDt1_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);
/*
** ===================================================================
**     Method      :  TmDt1_ParseCommand (component GenericTimeDate)
**     Description :
**         Shell Command Line parser
**     Parameters  :
**         NAME            - DESCRIPTION
**       * cmd             - Pointer to command line
**       * handled         - Pointer to variable which tells if
**                           the command has been handled or not
**       * io              - Pointer to I/O structure
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

void TmDt1_DeInit(void);
/*
** ===================================================================
**     Method      :  TmDt1_DeInit (component GenericTimeDate)
**     Description :
**         Deinitializes the driver.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void TmDt1_AddTicks(uint16_t nofTicks);
/*
** ===================================================================
**     Method      :  TmDt1_AddTicks (component GenericTimeDate)
**     Description :
**         Same as AddTick(), but multiple ticks can be added in one
**         step.
**     Parameters  :
**         NAME            - DESCRIPTION
**         nofTicks        - Number of ticks to be added
**     Returns     : Nothing
** ===================================================================
*/

/* END TmDt1. */

#endif
/* ifndef __TmDt1_H */
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
