/*
 * Shell.c
 *
 *  Created on: 04.08.2011
 *      Author: Erich Styger
 */

#include "Shell.h"
#include "Application.h"
#include "FRTOS1.h"
#include "CLS1.h"
#include "LEDR.h"
#include "LEDG.h"
#include "FAT1.h"

#define PL_HAS_SD_CARD  1 /* if we have SD card support */

static const CLS1_ParseCommandCallback CmdParserTable[] =
{
  CLS1_ParseCommand,
#if LEDR_PARSE_COMMAND_ENABLED
  LEDR_ParseCommand,
#endif
#if LEDG_PARSE_COMMAND_ENABLED
  LEDG_ParseCommand,
#endif
#if LEDB_PARSE_COMMAND_ENABLED
  LEDB_ParseCommand,
#endif
#if FRTOS1_PARSE_COMMAND_ENABLED
  FRTOS1_ParseCommand,
#endif
#if RTC1_PARSE_COMMAND_ENABLED
  RTC1_ParseCommand,
#endif
#if FAT1_PARSE_COMMAND_ENABLED
  FAT1_ParseCommand,
#endif
  BTS_ParseCommand,				//Battery Test System command parse table entry
  NULL /* sentinel */
};

static portTASK_FUNCTION(ShellTask, pvParameters) {
#if PL_HAS_SD_CARD
  bool cardMounted = FALSE;
  static FAT1_FATFS fileSystemObject;
#endif
  unsigned char buf[48];

  (void)pvParameters; /* not used */
  buf[0] = '\0';
  (void)CLS1_ParseWithCommandTable((unsigned char*)CLS1_CMD_HELP, CLS1_GetStdio(), CmdParserTable);
#if PL_HAS_SD_CARD
  FAT1_Init();
#endif
  for(;;) {
#if PL_HAS_SD_CARD
    (void)FAT1_CheckCardPresence(&cardMounted,
        "0" /* drive */, &fileSystemObject, CLS1_GetStdio());
    if (cardMounted) {
      //SD_GreenLed_On();
      //SD_RedLed_Off();
    } else {
      //SD_GreenLed_Off();
      //SD_RedLed_On();
    }
#endif
    (void)CLS1_ReadAndParseWithCommandTable(buf, sizeof(buf), CLS1_GetStdio(), CmdParserTable);
    FRTOS1_vTaskDelay(200/portTICK_RATE_MS);
    LEDG_Neg();
  }
}

void SHELL_Init(void) {
  if (FRTOS1_xTaskCreate(ShellTask, "Shell", configMINIMAL_STACK_SIZE+200, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
    for(;;){} /* error */
  }
}


/*
** ===================================================================
**     Method      :  BTS_ParseCommand
**     Description :
**         Shell Command Line parser for the Battery Test System. This
**         lists the commands recognized by the BTS.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * cmd             - Pointer to command string
**       * handled         - Pointer to variable which tells if
**                           the command has been handled or not
**       * io              - Pointer to I/O structure
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
byte FAT1_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io)
{
  if (UTIL1_strcmp((char*)cmd, "BTS help")==0) {
    *handled = TRUE;
    return PrintBTSHelp(io);
  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0) || (UTIL1_strcmp((char*)cmd, "FAT1 status")==0)) {
    *handled = TRUE;
    return PrintStatus(io);
  } else if (UTIL1_strncmp((char*)cmd, "FAT1 cd", sizeof("FAT1 cd")-1)==0) {
    *handled = TRUE;
    return CdCmd(cmd+sizeof("FAT1"), io);
  } else if (UTIL1_strncmp((char*)cmd, "FAT1 dir", sizeof("FAT1 dir")-1)==0) {
    *handled = TRUE;
    return DirCmd(cmd+sizeof("FAT1"), io);
  } else if (UTIL1_strncmp((char*)cmd, "FAT1 copy", sizeof("FAT1 copy")-1)==0) {
    *handled = TRUE;
    return CopyCmd(cmd+sizeof("FAT1"), io);
  } else if (UTIL1_strncmp((char*)cmd, "FAT1 delete", sizeof("FAT1 delete")-1)==0) {
    *handled = TRUE;
    return DeleteCmd(cmd+sizeof("FAT1"), io);
  } else if (UTIL1_strncmp((char*)cmd, "FAT1 mkdir", sizeof("FAT1 mkdir")-1)==0) {
    *handled = TRUE;
    return MkdirCmd(cmd+sizeof("FAT1"), io);
  } else if (UTIL1_strncmp((char*)cmd, "FAT1 rename", sizeof("FAT1 rename")-1)==0) {
    *handled = TRUE;
    return RenameCmd(cmd+sizeof("FAT1"), io);
  } else if (UTIL1_strncmp((char*)cmd, "FAT1 print", sizeof("FAT1 print")-1)==0) {
    *handled = TRUE;
    return PrintCmd(cmd+sizeof("FAT1"), io);
  } else if (UTIL1_strcmp((char*)cmd, "FAT1 diskinfo")==0) {
    *handled = TRUE;
    return FAT1_PrintDiskInfo(0, io);
  } else if (UTIL1_strncmp((char*)cmd, "FAT1 sector", sizeof("FAT1 sector")-1)==0) {
    *handled = TRUE;
    return SectorCmd(cmd+sizeof("FAT1"), io);
  } else if (UTIL1_strcmp((char*)cmd, "FAT1 benchmark")==0) {
    *handled = TRUE;
    return FAT1_Benchmark(io);
  }
  return ERR_OK;
}

