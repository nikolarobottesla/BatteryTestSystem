/*
 * Application.h
 *
 *      Original Author: Erich Styger
 *      Heavily Modified by: Milo Oien-Rochat
 */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

#include "CLS1.h"

#ifndef APPLICATION_H_
#define APPLICATION_H_

void APP_Run(void);

#endif /* APPLICATION_H_ */

//used to indicate system state
#define cycleDone -1
#define idleMode 0
#define IntResTest 1
#define chgMode 2
#define disMode 3

//used to indicate error type
#define noError 0
#define overCurrent 1

//define ADC channels
#define CHG_CURR 0
#define DIS_CURR 1
#define VTEMP 2
#define VBAT_SE 3

//define buffer array sizes for data to be written
#define dataBuffSize 30

//function prototypes
static void loadSettings(void);//
static void measureAll(void);
static void chkComplete(void);
static void writeData(void);
static void iteratePID(void);
static void stop_CHG_DIS(void);        //immediately stops charging or discharging
static void start_CHG_DIS(void);       //
byte BTS_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);
static uint8_t PrintHelp(const CLS1_StdIOType *io);
static uint8_t PrintStatus(const CLS1_StdIOType *io);


