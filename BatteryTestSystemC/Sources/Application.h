/*
 * Application.h
 *
 *      Author: Erich Styger
 */

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

//function prototypes
static void loadSettings(void);//
static void measure(void);
static void chkComplete(void);
static void writeData(void);
static void iteratePID(void);
static void stop_CHG_DIS(void);        //immediately stops charging or discharging
static void start_CHG_DIS(void);       //
