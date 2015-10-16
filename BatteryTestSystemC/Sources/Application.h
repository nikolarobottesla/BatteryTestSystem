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

//used to indicate system state, variable: state
#define CYCLE_DONE -1
#define IDLE_MODE 0
#define INT_RES_TEST 1
#define CHG_MODE 2
#define DIS_MODE 3

//used to indicate reason for stopping chg/dis
#define DELTA_PEAK 1
#define VBAT_CUT 2
#define TIMEOUT	3
#define OVER_TEMP 4
#define DROPOUT 5
#define OVER_CAPACITY 6
#define USER_COMMAND 7

//used to indicate error type, variable: error
#define NO_ERROR 0
#define OVER_CURRENT 1

//define ADC channels
#define CHG_CURR 0
#define DIS_CURR 1
#define VTEMP 2
#define VBAT_SE 3

//define buffer array sizes for data to be written
#define DATA_BUFF_SIZE 30

//function prototypes
static void State_Machine();
static void Load_Settings(void);//
static void Measure_All(void);
static void Chk_Complete(void);
static void Write_Data(void);
static void Iterate_PID(void);
static void Stop_CHG_DIS(void);        //immediately stops charging or discharging
static void Start_CHG_DIS(void);       //
byte BTS_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);
static uint8_t PrintHelp(const CLS1_StdIOType *io);
static uint8_t PrintStatus(const CLS1_StdIOType *io);
static void Load_Current(int newCurrent);


