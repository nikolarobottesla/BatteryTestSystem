/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : CLS1.c
**     Project     : BatteryTestSystemC
**     Processor   : MKL25Z128VLK4
**     Component   : Shell
**     Version     : Component 01.073, Driver 01.00, CPU db: 3.00.000
**     Repository  : mcuoneclipse
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-10-04, 10:11, # CodeGen: 30
**     Abstract    :
**
**     Settings    :
**          Component name                                 : CLS1
**          Echo                                           : no
**          Prompt                                         : "CMD> "
**          Project Name                                   : BatteryTestSystem
**          Silent Mode Prefix                             : #
**          Blocking Send                                  : Enabled
**            Wait                                         : WAIT1
**            Wait Time (ms)                               : 100
**            RTOS Wait                                    : yes
**          Status Colon Pos                               : 13
**          Help Semicolon Pos                             : 26
**          Multi Command                                  : Disabled
**          Utility                                        : UTIL1
**          Default Serial                                 : Enabled
**            Console Interface                            : AS1
**          Semaphore                                      : yes
**          Critical Section                               : CS1
**          History                                        : no
**     Contents    :
**         PrintPrompt                  - void CLS1_PrintPrompt(CLS1_ConstStdIOType *io);
**         SendNum8u                    - void CLS1_SendNum8u(uint8_t val, CLS1_StdIO_OutErr_FctType io);
**         SendNum8s                    - void CLS1_SendNum8s(int8_t val, CLS1_StdIO_OutErr_FctType io);
**         SendNum16u                   - void CLS1_SendNum16u(uint16_t val, CLS1_StdIO_OutErr_FctType io);
**         SendNum16s                   - void CLS1_SendNum16s(int16_t val, CLS1_StdIO_OutErr_FctType io);
**         SendNum32u                   - void CLS1_SendNum32u(uint32_t val, CLS1_StdIO_OutErr_FctType io);
**         SendNum32s                   - void CLS1_SendNum32s(int32_t val, CLS1_StdIO_OutErr_FctType io);
**         SendStr                      - void CLS1_SendStr(const uint8_t *str, CLS1_StdIO_OutErr_FctType io);
**         SendData                     - void CLS1_SendData(const uint8_t *data, uint16_t dataSize,...
**         PrintStatus                  - uint8_t CLS1_PrintStatus(CLS1_ConstStdIOType *io);
**         ParseCommand                 - uint8_t CLS1_ParseCommand(const uint8_t *cmd, bool *handled,...
**         IsHistoryCharacter           - bool CLS1_IsHistoryCharacter(uint8_t ch, uint8_t *cmdBuf, size_t cmdBufIdx,...
**         ReadLine                     - bool CLS1_ReadLine(uint8_t *bufStart, uint8_t *buf, size_t bufSize,...
**         PrintCommandFailed           - void CLS1_PrintCommandFailed(const uint8_t *cmd, CLS1_ConstStdIOType *io);
**         IterateTable                 - uint8_t CLS1_IterateTable(const uint8_t *cmd, bool *handled,...
**         SetStdio                     - uint8_t CLS1_SetStdio(CLS1_ConstStdIOTypePtr stdio);
**         GetStdio                     - CLS1_ConstStdIOTypePtr CLS1_GetStdio(void);
**         RequestSerial                - void CLS1_RequestSerial(void);
**         ReleaseSerial                - void CLS1_ReleaseSerial(void);
**         ReadAndParseWithCommandTable - uint8_t CLS1_ReadAndParseWithCommandTable(uint8_t *cmdBuf, size_t cmdBufSize,...
**         ParseWithCommandTable        - uint8_t CLS1_ParseWithCommandTable(const uint8_t *cmd, CLS1_ConstStdIOType...
**         GetSemaphore                 - void* CLS1_GetSemaphore(void);
**         SendStatusStr                - void CLS1_SendStatusStr(const uint8_t *strItem, const uint8_t *strStatus,...
**         SendHelpStr                  - void CLS1_SendHelpStr(const uint8_t *strCmd, const uint8_t *strHelp,...
**         ReadChar                     - void CLS1_ReadChar(uint8_t *c);
**         SendChar                     - void CLS1_SendChar(uint8_t ch);
**         KeyPressed                   - bool CLS1_KeyPressed(void);
**         Init                         - void CLS1_Init(void);
**         Deinit                       - void CLS1_Deinit(void);
**
**     License   :  Open Source (LGPL)
**     Copyright : (c) Copyright Erich Styger, 2014, all rights reserved.
**     http      : http://www.mcuoneclipse.com
**     This an open source software implementing a command line shell with Processor Expert.
**     This is a free software and is opened for education,  research  and commercial developments under license policy of following terms:
**     * This is a free software and there is NO WARRANTY.
**     * No restriction on use. You can use, modify and redistribute it for personal, non-profit or commercial product UNDER YOUR RESPONSIBILITY.
**     * Redistributions of source code must retain the above copyright notice.
** ###################################################################*/
/*!
** @file CLS1.c
** @version 01.00
** @brief
**
*/         
/*!
**  @addtogroup CLS1_module CLS1 module documentation
**  @{
*/         

/* MODULE CLS1. */
#include <ctype.h> /* for isalnum*/

#include "CLS1.h"
/* Include inherited components */
#include "WAIT1.h"
#include "UTIL1.h"
#include "AS1.h"
#include "CS1.h"

#if CLS1_HISTORY_ENABLED
  static uint8_t CLS1_history[CLS1_NOF_HISTORY][CLS1_HIST_LEN]; /* History buffers */
  static uint8_t CLS1_history_index = 0; /* Selected command */
#endif

#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __HC08__
  #pragma MESSAGE DISABLE C3303 /* implicit concatenation of strings */
#endif
static xSemaphoreHandle ShellSem = NULL; /* Semaphore to protect shell SCI access */

static CLS1_ConstStdIOType CLS1_stdio =
{
  (CLS1_StdIO_In_FctType)CLS1_ReadChar, /* stdin */
  (CLS1_StdIO_OutErr_FctType)CLS1_SendChar, /* stdout */
  (CLS1_StdIO_OutErr_FctType)CLS1_SendChar, /* stderr */
  CLS1_KeyPressed /* if input is not empty */
};
static CLS1_ConstStdIOType *CLS1_currStdIO = &CLS1_stdio;
/* Internal method prototypes */
static void SendSeparatedStrings(const uint8_t *strA, const uint8_t *strB, uint8_t tabChar, uint8_t tabPos, CLS1_StdIO_OutErr_FctType io);



/*
** ===================================================================
**     Method      :  CLS1_SendStr (component Shell)
**     Description :
**         Prints a string using an I/O function
**     Parameters  :
**         NAME            - DESCRIPTION
**       * str             - Pointer to string (zero terminated) to be
**                           printed.
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
/*!
 * \brief Prints a string using I/O callbacks
 * \param[in] str String (zero terminated) to be printed
 * \param[in] io I/O function to be used for printing
 */
void CLS1_SendStr(const uint8_t *str, CLS1_StdIO_OutErr_FctType io)
{
  while(*str!='\0') {
    io(*str++);
  }
}

/*
** ===================================================================
**     Method      :  CLS1_SendNum32s (component Shell)
**     Description :
**         Sends a 32bit signed number to the given I/O
**     Parameters  :
**         NAME            - DESCRIPTION
**         val             - number to print
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_SendNum32s(int32_t val, CLS1_StdIO_OutErr_FctType io)
{
  unsigned char buf[sizeof("-1234567890")];

  UTIL1_Num32sToStr(buf, sizeof(buf), val);
  CLS1_SendStr(buf, io);
}

/*
** ===================================================================
**     Method      :  CLS1_SendNum32u (component Shell)
**     Description :
**         Sends a 32bit unsigned number to the given I/O
**     Parameters  :
**         NAME            - DESCRIPTION
**         val             - number to print
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_SendNum32u(uint32_t val, CLS1_StdIO_OutErr_FctType io)
{
  unsigned char buf[sizeof("1234567890")];

  UTIL1_Num32uToStr(buf, sizeof(buf), val);
  CLS1_SendStr(buf, io);
}

/*
** ===================================================================
**     Method      :  CLS1_SendNum16s (component Shell)
**     Description :
**         Sends a 16bit signed number to the given I/O
**     Parameters  :
**         NAME            - DESCRIPTION
**         val             - number to print
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_SendNum16s(int16_t val, CLS1_StdIO_OutErr_FctType io)
{
  unsigned char buf[sizeof("-12345")];

  UTIL1_Num16sToStr(buf, sizeof(buf), val);
  CLS1_SendStr(buf, io);
}

/*
** ===================================================================
**     Method      :  CLS1_SendNum16u (component Shell)
**     Description :
**         Sends a 16bit unsigned number to the given I/O
**     Parameters  :
**         NAME            - DESCRIPTION
**         val             - number to print
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_SendNum16u(uint16_t val, CLS1_StdIO_OutErr_FctType io)
{
  unsigned char buf[sizeof("12345")];

  UTIL1_Num16uToStr(buf, sizeof(buf), val);
  CLS1_SendStr(buf, io);
}

/*
** ===================================================================
**     Method      :  CLS1_SendNum8u (component Shell)
**     Description :
**         Sends an 8bit unsigned number to the given I/O
**     Parameters  :
**         NAME            - DESCRIPTION
**         val             - number to print
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_SendNum8u(uint8_t val, CLS1_StdIO_OutErr_FctType io)
{
  unsigned char buf[sizeof("123")];

  UTIL1_Num8uToStr(buf, sizeof(buf), val);
  CLS1_SendStr(buf, io);
}

/*
** ===================================================================
**     Method      :  CLS1_SendNum8s (component Shell)
**     Description :
**         Sends an 8bit signed number to the given I/O
**     Parameters  :
**         NAME            - DESCRIPTION
**         val             - number to print
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_SendNum8s(int8_t val, CLS1_StdIO_OutErr_FctType io)
{
  unsigned char buf[sizeof("-123")];

  UTIL1_Num8sToStr(buf, sizeof(buf), val);
  CLS1_SendStr(buf, io);
}

/*
** ===================================================================
**     Method      :  CLS1_ParseCommand (component Shell)
**     Description :
**         Parses a shell command. Use 'help' to get a list of
**         supported commands.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * cmd             - Pointer to command string
**       * handled         - Pointer to variable to indicate if
**                           the command has been handled. The caller
**                           passes this variable to the command scanner
**                           to find out if the passed command has been
**                           handled. The variable is initialized by the
**                           caller.
**       * io              - Pointer to I/O callbacks
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t CLS1_ParseCommand(const uint8_t *cmd, bool *handled, CLS1_ConstStdIOType *io)
{
  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, "CLS1 help")==0) {
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
    CLS1_SendStr((unsigned char*)CLS1_DASH_LINE, io->stdOut);
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
    CLS1_SendStr((unsigned char*)"BatteryTestSystem", io->stdOut);
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
    CLS1_SendStr((unsigned char*)CLS1_DASH_LINE, io->stdOut);
    CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"CLS1", (const unsigned char*)"Group of CLS1 commands\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  help|status", (const unsigned char*)"Print help or status information\r\n", io->stdOut);
    *handled = TRUE;
    return ERR_OK;
  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0) || (UTIL1_strcmp((char*)cmd, "CLS1 status")==0)) {
    *handled = TRUE;
    return CLS1_PrintStatus(io);
  }
  return ERR_OK; /* no error */
}

/*
** ===================================================================
**     Method      :  CLS1_PrintPrompt (component Shell)
**     Description :
**         Prints the prompt to the stdOut channel
**     Parameters  :
**         NAME            - DESCRIPTION
**       * io              - Pointer to IO to be used
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_PrintPrompt(CLS1_ConstStdIOType *io)
{
  CLS1_SendStr((unsigned char*)"CMD> ", io->stdOut);
}

/*
** ===================================================================
**     Method      :  CLS1_IsHistoryCharacter (component Shell)
**     Description :
**         Returns TRUE if character is a history character
**     Parameters  :
**         NAME            - DESCRIPTION
**         ch              - current command character
**       * cmdBuf          - Pointer to command line buffer read
**                           so far
**         cmdBufIdx       - Index of character into cmdBuf
**       * isPrev          - Pointer to return value, if it is
**                           'previous' history or not
**     Returns     :
**         ---             - TRUE if it is an accepted history character
** ===================================================================
*/
bool CLS1_IsHistoryCharacter(uint8_t ch, uint8_t *cmdBuf, size_t cmdBufIdx, bool *isPrev)
{
  *isPrev = FALSE;
#if CLS1_HISTORY_ENABLED
  if (   cmdBufIdx==0 /* first character on command line */
      || (UTIL1_strcmp(cmdBuf, CLS1_history[CLS1_history_index])==0) /* pressing prev/next character on previous history element */
      )
  {
    if (ch==CLS1_HISTORY_PREV_CHAR) {
      *isPrev = TRUE;
      return TRUE;
    } else if (ch==CLS1_HISTORY_NEXT_CHAR) {
      *isPrev = FALSE;
      return TRUE;
    }
  }
#if 0
  if (cmdBufIdx==0 || cmdBufIdx==2) { /* accept only first character or sequence as history sequence */
    if (cmdBufIdx==2 && cmdBuf[0]==0x1b && cmdBuf[1]==0x5b) {
      /* up:    0x27 0x5b 0x41
       * down:  0x27 0x5b 0x42
       * right: 0x27 0x5b 0x43
       * left:  0x27 0x5b 0x44
       */
      if (cmdBuf[2]==0x41 /* up */ || cmdBuf[2]==0x44 /* left */) {
        *isPrev = TRUE;
        return TRUE;
      } else if (cmdBuf[2]==0x42 /* down */ || cmdBuf[2]==0x43 /* right */) {
        *isPrev = FALSE;
        return TRUE;
      }
    }
    /* \todo: handle TAB and SHIFT-TAB */
  }
#endif
#endif
  return FALSE;
}

/*
** ===================================================================
**     Method      :  CLS1_ReadLine (component Shell)
**     Description :
**         Reads a line from stdIn and returns TRUE if we have a line,
**         FALSE otherwise.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * bufStart        - Pointer to start of buffer
**       * buf             - Pointer to buffer where to read in the
**                           information
**         bufSize         - size of buffer
**       * io              - Pointer to I/O callbacks
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
bool CLS1_ReadLine(uint8_t *bufStart, uint8_t *buf, size_t bufSize, CLS1_ConstStdIOType *io)
{
  uint8_t c;
  bool isBackwardHistory;

  if (io->keyPressed()) {
    for(;;) {                          /* while not '\r' or '\n' */
      c = '\0';                        /* initialize character */
      io->stdIn(&c);                   /* read character */
      if (c=='\0') { /* nothing in rx buffer? Something is wrong... */
        break; /* get out of loop */
      }
      if (c=='\b' || c=='\177') {      /* check for backspace */
        if (buf > bufStart) {          /* Avoid buffer underflow */
#if CLS1_ECHO_ENABLED
           io->stdOut('\b');           /* delete character on terminal */
           io->stdOut(' ');
           io->stdOut('\b');
#endif
           buf--;                      /* delete last character in buffer */
           *buf = '\0';
           bufSize++;
        }
      } else if (CLS1_IsHistoryCharacter(c, bufStart, buf-bufStart, &isBackwardHistory)) {
#if CLS1_HISTORY_ENABLED
        uint8_t cBuf[3]={'\0','\0','\0'}, cBufIdx = 0;
        bool prevInHistory;
#endif

        while (c!='\0') {              /* empty the rx buffer (escape sequence) */
#if CLS1_HISTORY_ENABLED
           cBuf[cBufIdx] = c;
           cBufIdx++;
           if (cBufIdx==sizeof(cBuf)) {
             cBufIdx = 0; /* ring buffer */
           }
#endif
           c = '\0';                   /* initialize character */
           io->stdIn(&c);              /* read character */
        }
#if CLS1_HISTORY_ENABLED
        /* if not an alphanumeric switch to history  */
        prevInHistory = cBufIdx==0 && cBuf[0]==0x1b && cBuf[1]==0x5b && (cBuf[2]==0x41 /*up*/ || cBuf[2]==0x44 /*left*/);
        /* up:    0x27 0x5b 0x41
         * down:  0x27 0x5b 0x42
         * right: 0x27 0x5b 0x43
         * left:  0x27 0x5b 0x44
         */
        if (prevInHistory) {
          UTIL1_strcpy(bufStart, CLS1_HIST_LEN, CLS1_history[CLS1_history_index]);
          CLS1_history_index++;        /* update the index */
          if (CLS1_history_index==CLS1_NOF_HISTORY) {
            CLS1_history_index = 0;
          }
        } else {
          if (CLS1_history_index==0) {
            CLS1_history_index = (CLS1_NOF_HISTORY-1);
          } else {
            CLS1_history_index--;
          }
          UTIL1_strcpy(bufStart, CLS1_HIST_LEN, CLS1_history[CLS1_history_index]);
        }
        bufSize = bufSize + buf - bufStart - UTIL1_strlen(bufStart); /* update the buffer */
        buf = bufStart + UTIL1_strlen(bufStart);
#endif
#if CLS1_ECHO_ENABLED
        CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
        CLS1_PrintPrompt(io);
        CLS1_SendStr(bufStart, io->stdOut);
#endif
      } else {
#if CLS1_ECHO_ENABLED
        io->stdOut(c);                 /* echo character */
#endif
        *buf = (uint8_t)c;             /* append character to the string */
        buf++;
        bufSize--;
        if ((c=='\r') || (c=='\n')) {
#if CLS1_ECHO_ENABLED
          CLS1_SendStr((unsigned char*)"\n", io->stdOut);
#endif
#if CLS1_HISTORY_ENABLED
          if ((bufStart[0] != '\0') && (bufStart[0] != '\r') && (bufStart[0] != '\n')) {
            int i;

            for(i=CLS1_NOF_HISTORY-1; i>0;i--) {
              UTIL1_strcpy(CLS1_history[i], CLS1_HIST_LEN, CLS1_history[i-1]); /* move previous commands */
            }
            CLS1_history_index = 0;    /* update the history with the current command */
            UTIL1_strcpy(CLS1_history[0], CLS1_HIST_LEN, bufStart); /* add the current command to the history */
            if (buf-bufStart <= CLS1_HIST_LEN) { /* size check */
              CLS1_history[0][buf-bufStart-1] = '\0';
            } else {
              CLS1_history[0][CLS1_HIST_LEN-1] = '\0';
            }
          }
#endif
          break;
        }
        if (bufSize <= 1) {            /* buffer full */
          break;
        }
      }
    } /* for */
    *buf = '\0';                       /* zero terminate string */
    return TRUE;
  } else {
    return FALSE;
  }
}

/*
** ===================================================================
**     Method      :  CLS1_PrintStatus (component Shell)
**     Description :
**         Prints various available system status information
**     Parameters  :
**         NAME            - DESCRIPTION
**       * io              - Pointer to I/O callbacks
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t CLS1_PrintStatus(CLS1_ConstStdIOType *io)
{
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  CLS1_SendStr((unsigned char*)CLS1_DASH_LINE, io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\nSYSTEM STATUS\r\n", io->stdOut);
  CLS1_SendStr((unsigned char*)CLS1_DASH_LINE, io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  CLS1_SendStatusStr((const unsigned char*)"Firmware", (const unsigned char*)__DATE__, io->stdOut);
  CLS1_SendStr((unsigned char*)" ", io->stdOut);
  CLS1_SendStr((unsigned char*)__TIME__, io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  return ERR_OK;
}

/*
** ===================================================================
**     Method      :  CLS1_PrintCommandFailed (component Shell)
**     Description :
**         Prints a standard message for failed or unknown commands
**     Parameters  :
**         NAME            - DESCRIPTION
**       * cmd             - Pointer to command which was failing
**       * io              - Pointer to I/O callbacks
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_PrintCommandFailed(const uint8_t *cmd, CLS1_ConstStdIOType *io)
{
  CLS1_SendStr((unsigned char*)"*** Failed or unknown command: ", io->stdErr);
  CLS1_SendStr(cmd, io->stdErr);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdErr);
  CLS1_SendStr((unsigned char*)"*** Type ", io->stdErr);
  CLS1_SendStr((unsigned char*)CLS1_CMD_HELP, io->stdErr);
  CLS1_SendStr((unsigned char*)" to get a list of available commands\r\n", io->stdErr);
}

/*
** ===================================================================
**     Method      :  CLS1_IterateTable (component Shell)
**     Description :
**         Parses a shell command. It handles first the internal
**         commands and will call the provided callback.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * cmd             - Pointer to command string
**       * handled         - Pointer to boolean which is set to
**                           TRUE if a command parser has handled the
**                           command.
**       * io              - Pointer to I/O callbacks
**       * parserTable     - Pointer to callback which
**                           will be called to parse commands in the
**                           user application, or NULL if not used.
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t CLS1_IterateTable(const uint8_t *cmd, bool *handled, CLS1_ConstStdIOType *io, CLS1_ConstParseCommandCallback *parserTable)
{
  uint8_t res = ERR_OK;

  if (parserTable==NULL) { /* no table??? */
    return ERR_FAILED;
  }
  /* iterate through all parser functions in table */
  while(*parserTable!=NULL) {
    if ((*parserTable)(cmd, handled, io)!=ERR_OK) {
      res = ERR_FAILED;
    }
    parserTable++;
  }
  return res;
}

/*
** ===================================================================
**     Method      :  CLS1_ParseWithCommandTable (component Shell)
**     Description :
**         Parses a shell command. It handles first the internal
**         commands and will call the provided callback.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * cmd             - Pointer to command string
**       * io              - Pointer to I/O callbacks
**       * parseCallback   - Pointer to callback
**                           which will be called to parse commands in
**                           the user application, or NULL if not used.
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t CLS1_ParseWithCommandTable(const uint8_t *cmd, CLS1_ConstStdIOType *io, CLS1_ConstParseCommandCallback *parseCallback)
{
  uint8_t res = ERR_OK;
  bool handled;
#if CLS1_SILENT_PREFIX_CHAR_ENABLED
  bool silent = FALSE;
#endif
#if CLS1_MULTI_CMD_ENABLED
  uint8_t buf[CLS1_MULTI_CMD_SIZE];
  uint8_t i;
  bool parseBuffer, finished;
#endif

  if (*cmd=='\0') { /* empty command */
    return ERR_OK;
  }
#if CLS1_MULTI_CMD_ENABLED
  parseBuffer = FALSE;
  finished = FALSE;
  i = 0;
  for(;;) { /* breaks */
    if (i>sizeof(buf)-2) {
      res = ERR_FAILED;
      break; /* buffer overflow */
    }
    buf[i] = *cmd;
    cmd++; i++;
  #if CLS1_SILENT_PREFIX_CHAR_ENABLED
    if (i==1 && buf[0]==CLS1_SILENT_PREFIX_CHAR) { /* first character is silent character */
      silent |= (bool)(buf[0]==CLS1_SILENT_PREFIX_CHAR);
      buf[0] = *cmd; /* skip silent character */
      cmd++;
    }
  #endif
    if (buf[i-1] == CLS1_MULTI_CMD_CHAR) { /* found separator */
      buf[i-1] = '\0';
      parseBuffer = TRUE;
    } else if (buf[i-1]=='\0') {
      parseBuffer = TRUE;
      finished = TRUE;
    }
    if (parseBuffer) {
      handled = FALSE;
      res = CLS1_IterateTable(buf, &handled, io, parseCallback); /* iterate through all parser functions in table */
      if (!handled || res!=ERR_OK) { /* no handler has handled the command, or error? */
        CLS1_PrintCommandFailed(buf, io);
        res = ERR_FAILED;
      }
      parseBuffer = FALSE;
      i = 0; /* restart */
    }
    if (finished) {
      break; /* get out of loop */
    }
  } /* for */
#else
  #if CLS1_SILENT_PREFIX_CHAR_ENABLED
  silent = (bool)(*cmd==CLS1_SILENT_PREFIX_CHAR);
  if (silent) {
    cmd++; /* skip silent character */
  }
  #endif
  handled = FALSE;
  res = CLS1_IterateTable(cmd, &handled, io, parseCallback); /* iterate through all parser functions in table */
  if (!handled || res!=ERR_OK) { /* no handler has handled the command? */
    CLS1_PrintCommandFailed(cmd, io);
    res = ERR_FAILED;
  }
#endif
#if CLS1_SILENT_PREFIX_CHAR_ENABLED
  if (!silent) {
    CLS1_PrintPrompt(io);
  }
#else
  CLS1_PrintPrompt(io);
#endif
  return res;
}

/*
** ===================================================================
**     Method      :  CLS1_SetStdio (component Shell)
**     Description :
**         Sets an StdIO structure which is returned by GetStdio()
**     Parameters  :
**         NAME            - DESCRIPTION
**         stdio           - New stdio structure to be used.
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t CLS1_SetStdio(CLS1_ConstStdIOTypePtr stdio)
{
  CLS1_currStdIO = stdio;
  return ERR_OK;
}

/*
** ===================================================================
**     Method      :  CLS1_GetStdio (component Shell)
**     Description :
**         Returns the default stdio channel. This method is only
**         available if a shell is enabled in the component properties.
**     Parameters  : None
**     Returns     :
**         ---             - Pointer to the stdio descriptor
** ===================================================================
*/
CLS1_ConstStdIOTypePtr CLS1_GetStdio(void)
{
  return CLS1_currStdIO;
}

/*
** ===================================================================
**     Method      :  CLS1_ReadAndParseWithCommandTable (component Shell)
**     Description :
**         Reads characters from the default input channel and appends
**         it to the buffer. Once a new line has been detected, the
**         line will be parsed using the handlers in the table.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * cmdBuf          - Pointer to buffer provided by the
**                           caller where to store the command to read
**                           in. Characters will be appended, so make
**                           sure string buffer is initialized with a
**                           zero byte at the beginning.
**         cmdBufSize      - Size of buffer
**       * io              - Pointer to I/O channels to be used
**       * parseCallback   - Pointer to callback
**                           table provided by the user application to
**                           parse commands. The table has a NULL
**                           sentinel.
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t CLS1_ReadAndParseWithCommandTable(uint8_t *cmdBuf, size_t cmdBufSize, CLS1_ConstStdIOType *io, CLS1_ConstParseCommandCallback *parseCallback)
{
  uint8_t res = ERR_OK;
  size_t len;

  /* IMPORTANT NOTE: this function *appends* to the buffer, so the buffer needs to be initialized first! */
  len = UTIL1_strlen((const char*)cmdBuf);
  if (CLS1_ReadLine(cmdBuf, cmdBuf+len, cmdBufSize-len, io)) {
    len = UTIL1_strlen((const char*)cmdBuf); /* length of buffer string */
    if (len==0) { /* error case */
      return ERR_FAILED;
    } else if (len==1 && (cmdBuf[0]=='\n' || cmdBuf[0]=='\r')) { /* eat preceding newline characters */
      cmdBuf[0] = '\0';
    }
    if (len>=cmdBufSize-1) {           /* buffer overflow? Parse what we have, will be likely return an error */
      (void)CLS1_ParseWithCommandTable(cmdBuf, io, parseCallback);
      cmdBuf[0] = '\0'; /* start again */
      res = ERR_OVERFLOW;
    } else if (cmdBuf[len-1]=='\n' || cmdBuf[len-1]=='\r') { /* line end: parse command */
      cmdBuf[len-1] = '\0';            /* remove line end character for parser */
      res = CLS1_ParseWithCommandTable(cmdBuf, io, parseCallback);
      cmdBuf[0] = '\0';                /* start again */
    } else {
      /* continue to append to buffer */
    }
  }
  return res;
}

/*
** ===================================================================
**     Method      :  CLS1_RequestSerial (component Shell)
**     Description :
**         Used to get mutual access to the shell console. Only has an
**         effect if using an RTOS with semaphore for the console
**         access.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_RequestSerial(void)
{
  (void)xSemaphoreTakeRecursive(ShellSem, portMAX_DELAY);
}

/*
** ===================================================================
**     Method      :  CLS1_ReleaseSerial (component Shell)
**     Description :
**         Used to release mutual access to the shell console. Only has
**         an effect if using an RTOS with semaphore for the console
**         access.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_ReleaseSerial(void)
{
  (void)xSemaphoreGiveRecursive(ShellSem);
}

/*
** ===================================================================
**     Method      :  CLS1_GetSemaphore (component Shell)
**     Description :
**         Return the semaphore of the shell.
**     Parameters  : None
**     Returns     :
**         ---             - semaphore, or NULL if not used or not
**                           allocated.
** ===================================================================
*/
void* CLS1_GetSemaphore(void)
{
  return ShellSem;
}

/*
** ===================================================================
**     Method      :  SendSeparatedStrings (component Shell)
**
**     Description :
**         Prints a string using an I/O function, formated for the 'help' 
**         command
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
static void SendSeparatedStrings(const uint8_t *strA, const uint8_t *strB, uint8_t tabChar, uint8_t tabPos, CLS1_StdIO_OutErr_FctType io)
{
  /* write command part */
  while(*strA!='\0' && tabPos>0) {
    io(*strA++);
    tabPos--;
  }
  /* fill up until ';' */
  while(tabPos>0) {
    io(' ');
    tabPos--;
  }
  /* write separator */
  io(tabChar);
  io(' ');
  /* write help text */
  CLS1_SendStr(strB, io);
}

/*
** ===================================================================
**     Method      :  CLS1_SendHelpStr (component Shell)
**     Description :
**         Prints a string using an I/O function, formated for the
**         'help' command
**     Parameters  :
**         NAME            - DESCRIPTION
**       * strCmd          - Pointer to string of the command
**       * strHelp         - Pointer to help text string
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_SendHelpStr(const uint8_t *strCmd, const uint8_t *strHelp, CLS1_StdIO_OutErr_FctType io)
{
  #define HELP_SEMICOLON_POS  26 /* position of the ';' after the command string */
  SendSeparatedStrings(strCmd, strHelp, ';', HELP_SEMICOLON_POS, io);
}

/*
** ===================================================================
**     Method      :  CLS1_SendStatusStr (component Shell)
**     Description :
**         Prints a status string using an I/O function, formated for
**         the 'status' command
**     Parameters  :
**         NAME            - DESCRIPTION
**       * strItem         - Pointer to string of the command
**       * strStatus       - Pointer to help text string
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_SendStatusStr(const uint8_t *strItem, const uint8_t *strStatus, CLS1_StdIO_OutErr_FctType io)
{
  #define STATUS_COLON_POS  13 /* position of the ':' after the item string */
  SendSeparatedStrings(strItem, strStatus, ':', STATUS_COLON_POS, io);
}

/*
** ===================================================================
**     Method      :  CLS1_ReadChar (component Shell)
**     Description :
**         Reads a character (blocking)
**     Parameters  :
**         NAME            - DESCRIPTION
**       * c               - Pointer to character to be used to store the
**                           result
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_ReadChar(uint8_t *c)
{
  uint8_t res;

  (void)xSemaphoreTakeRecursive(ShellSem, portMAX_DELAY);
  res = AS1_RecvChar((uint8_t*)c);
  if (res==ERR_RXEMPTY) {
    /* no character in buffer */
    *c = '\0';
  }
  (void)xSemaphoreGiveRecursive(ShellSem);
}

/*
** ===================================================================
**     Method      :  CLS1_SendChar (component Shell)
**     Description :
**         Sends a character (blocking)
**     Parameters  :
**         NAME            - DESCRIPTION
**         ch              - character to be sent
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_SendChar(uint8_t ch)
{
  uint8_t res;

  (void)xSemaphoreTakeRecursive(ShellSem, portMAX_DELAY);
  do {
    res = AS1_SendChar((uint8_t)ch);   /* Send char */
    if (res==ERR_TXFULL) {
      WAIT1_WaitOSms(100);
    }
  } while(res==ERR_TXFULL);
  (void)xSemaphoreGiveRecursive(ShellSem);
}

/*
** ===================================================================
**     Method      :  CLS1_KeyPressed (component Shell)
**     Description :
**         Checks if a key has been pressed (a character is present in
**         the input buffer)
**     Parameters  : None
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
bool CLS1_KeyPressed(void)
{
  bool res;

  (void)xSemaphoreTakeRecursive(ShellSem, portMAX_DELAY);
  res = (bool)((AS1_GetCharsInRxBuf()==0U) ? FALSE : TRUE); /* true if there are characters in receive buffer */
  (void)xSemaphoreGiveRecursive(ShellSem);
  return res;
}

/*
** ===================================================================
**     Method      :  CLS1_Init (component Shell)
**     Description :
**         Initializes the module, especially creates the mutex
**         semaphore if an RTOS is used.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_Init(void)
{
  bool schedulerStarted;
  CS1_CriticalVariable();

  schedulerStarted = (bool)(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED);
  if (!schedulerStarted) { /* FreeRTOS not started yet. We are called in PE_low_level_init(), and interrupts are disabled */
    CS1_EnterCritical();
  }
  ShellSem = xSemaphoreCreateRecursiveMutex();
  if (!schedulerStarted) { /* above RTOS call might have enabled interrupts! Make sure we restore the state */
    CS1_ExitCritical();
  }
  if (ShellSem==NULL) { /* semaphore creation failed */
    for(;;) {} /* error, not enough memory? */
  }
  vQueueAddToRegistry(ShellSem, "CLS1_Sem");
#if CLS1_HISTORY_ENABLED
  {
    int i;

    CLS1_history_index = 0;
    for(i=0; i<CLS1_NOF_HISTORY;i++) {
      CLS1_history[i][0] = '\0';
    }
  }
#endif
}

/*
** ===================================================================
**     Method      :  CLS1_Deinit (component Shell)
**     Description :
**         De-Initializes the module, especially frees the mutex
**         semaphore if an RTOS is used.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void CLS1_Deinit(void)
{
  vQueueUnregisterQueue(ShellSem);
  vSemaphoreDelete(ShellSem);
  ShellSem = NULL;
}

/*
** ===================================================================
**     Method      :  CLS1_SendData (component Shell)
**     Description :
**         Sends data using an I/O function. Unlike SendStr(), with
**         this method it is possible to send binary data, including
**         zero bytes.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * data            - Pointer to data to be sent
**         dataSize        - Number of bytes to be sent.
**         io              - I/O callbacks to be used for printing.
**     Returns     : Nothing
** ===================================================================
*/
/*!
 * \brief Sends data using I/O callbacks
 * \param[in] data Pointer to data to be sent
 * \param[in] io I/O function to be used for sending
 */
void CLS1_SendData(const uint8_t *data, uint16_t dataSize, CLS1_StdIO_OutErr_FctType io)
{
  while(dataSize>0) {
    io(*data++);
    dataSize--;
  }
}

/* END CLS1. */

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
