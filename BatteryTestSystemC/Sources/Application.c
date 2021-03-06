/*
 * Application.c
 *      Author: Milo Oien-Rochat
 */
#include "Application.h"
#include "LEDR.h"
#include "LEDG.h"
#include "FRTOS1.h"
#include "Shell.h"
#include "CHG_PWM.h"
#include "DIS_PWM.h"
#include "AD1.h"
#include "WAIT1.h"
#include "CS1.h"
#include "TmDt1.h"
#include "FAT1.h"

/* Global Variales */
static volatile int state=IDLE_MODE;//used to control the run mode of the system IDLE_MODE, INT_RES_TEST, CHG_MODE, DIS_MODE, CYCLE_DONE
static int maxCycles = 2;			//number of charge/discharge cycles
static int firstState = CHG_MODE;	//set to start charge or discharge as the first cycle
static volatile int doneReason=0;	//reason the cycle ended, see header file for macro
static int verbose=0;				//if set to 1 the status is printed every loop
static int logEnabled = 0;			//if set to 1 the status data is saved to a log file on uSD card
static volatile int chgCurrSet=0;	//charge current in mA
static volatile int disCurrSet=0;	//discharge current in mA
static float currentLimit=6500;		//current limit in mA
static float maxCurrSense=11000;	//the maximum current the adc can sense in mA
static int deltaPeak=24;			//delta peak charge cutoff voltage in mV (6 cells * 4mV)
static int disVcut=5400;			//discharge voltage cutoff in mV
static float VBatRateLimit = -10;	//cell dropout rate [mV/s]
//TODO change charge timeout back to longer time
static int chgTimeout = 1;			//charge cycle timeout in minutes
static int disTimeout = 120;		//discharge cycle timeout in minutes
static TickType_t interval = 1000;	//measurement interval in ms
static unsigned int waitTime = 5;	//wait time between cycles in minutes
static int chgCapLimit = 10000;		//charge capacity limit [mAh]
static int disCapLimit = 8000;		//discharge capacity limit [mAh]
static int nextState = 0;			//state machine tells the start routine the next state using this variable
static int cyclesLeft = 0;			//number of cycles left over not including current cycle
static float capacity;				//accumulates capacity [mAh]
int intCapacity;				 	//capcity converted to int type, used for displaying status and writing to file
static float OTthresh = 50;        	//over temperature threshhold in celsius
static float PID_Kp = 0.1;          //Proportional constant
static float PID_Ki = 0.05;         //Integral constant
//float Kd = 0.1;             //Derivative constant
//float prevError = 0;        //previous error value
//float difError;             //Differential error, difError - err from previous loop'
static float perCurrent;			//float, stores the battery current read by the ADC, between 0 and 1
static int current[DATA_BUFF_SIZE];	//int array, the battery current in mA, saved to uSD card
static int VBat[DATA_BUFF_SIZE];	//int array, stores the battery voltage in mV, saved to uSD card
static int temp[DATA_BUFF_SIZE];	//int array, stores the temperature in C, saved to uSD card
static int timeStamp[DATA_BUFF_SIZE]={-1};	//stores the time a measurement was made, saved to uSD card
static int lastStamp;				//stores last loop's timestamp
static int dp;						//data pointer, used to indicate which element is most recent in the measured data arrays
static float periodSec;				//period between ADC measurements in seconds
static uint16_t rawADC[AD1_CHANNEL_COUNT];	//stores latest adc measurements
static int maxVBat;					//maximum battery voltage during charge cycle, mV
static float deltaVBat;				//difference between the maximum and most recently measured battery voltage, Volts
static float preVBat;				//stores the previous Vbat measurement, Volts
static float rateVBat;				//the battery voltage rate, used for cell dropout check [volts/sec]
static float PID_perCurrentSet;		//float, stores the battery current setting scaled between 0 and 1
static float PID_perCurrentLimit;	//float, stores current limit scaled between 0 and 1
static float PID_intError;			//integration result from previous loop + error * periodSec
static char charBuff[64];			//buffer used for io
TickType_t xLastWakeTime;			//Declare the xLastWakeTime variable
static FIL fp;						//declare a file pointer, used to save data to file

/*OS task not used
static portTASK_FUNCTION(R_LEDblink, pvParameters) {
  (void)pvParameters; // parameter not used

  for(;;) {
    //LEDR_Neg();
    FRTOS1_vTaskDelay(5000/portTICK_RATE_MS);
  }
}*/

static portTASK_FUNCTION(control, pvParameters) {
  (void)pvParameters; /* parameter not used */

  //initialization
  CHG_PWM_Enable();
  DIS_PWM_Enable();
  Stop_CHG_DIS();
  AD1_Calibrate(1);	//calibrate adc and wait till done


  for(;;) {

	  if (state==CYCLE_DONE){	//if this is the start of a new cycle
		  Load_Settings();			//load settings
	  }

	  while(state >= CHG_MODE){
		  LEDR_Neg();

		  timeStamp[dp] = lastStamp + 1;	//increment timestamp

		  Measure_All();
//TODO	  //Chk_Complete();
		  Iterate_PID();

		  if (logEnabled == 1){		//if log is enabled
			  Log_To_File();				//log to file on uSD card
		  }

		  if (verbose == 1){		//print status if verbose is set to 1
			PrintStatus(CLS1_GetStdio());
		  }

		  lastStamp = timeStamp[dp];	//save the last time stamp for use in the next loop iteration
		  dp++;							//increment pointer for data buffers
		  if (dp >= DATA_BUFF_SIZE){
			  dp=0;						//reset pointer for data buffers
		  }

		  FRTOS1_vTaskDelayUntil(&xLastWakeTime, interval/portTICK_RATE_MS);	//using this vs TaskDelay to get a specific period
	  }

	  Stop_CHG_DIS();	//needed if cycle was stopped manually, also just to make double sure the PWM channels are off

	  if ( doneReason > 0 ){
		  State_Machine(CLS1_GetStdio());}		//choose what to do next

	  FRTOS1_vTaskDelay(5000/portTICK_RATE_MS);
  }
}

//function to stop charging or discharging, PWMs are active low
void Stop_CHG_DIS(void)
{
    CHG_PWM_SetRatio16(0xFFFF);	//turn off CHG PFETs to disconnect 12V
    DIS_PWM_SetRatio16(0xFFFF); //turn off discharge NFET if not already off to disconect the load from the circuit.
}

static void Load_Settings(void){

	if (state == CHG_MODE){
		Load_Current(chgCurrSet);
	}else if (state == DIS_MODE){
		Load_Current(disCurrSet);
	}

    PID_perCurrentLimit = currentLimit / maxCurrSense;	//calculate percent current limit

	cyclesLeft = cyclesLeft - 1;
	state = nextState;

    dp = 0;									//initialize data pointer
    lastStamp = 0;							//initialize last time stamp
	timeStamp[0] = 0;						//set initial timeStamp
    maxVBat=0;								//initialize to 0mV
    preVBat=0;								//initialize at 0mV to prevent cell dropout detector from triggering on 1st check
    periodSec = interval / 1000;			//calculate the measure period in seconds
    PID_intError = 0;

    xLastWakeTime= xTaskGetTickCount(); 	//reload xLastWakeTime variable with the current time, this needs to be loaded
    										//just prior to starting the cycle, if loaded before cycle start the first few
    										//loops of the while loop will happen very fast
}

static void Measure_All(void){
	AD1_Measure(1);					//start adc conversion and wait till complete
	AD1_GetValue16(&rawADC[0]);		//put ADC values in array rawADC
	VBat[dp] = rawADC[VBAT_SE] / (float)0xFFFF * 9900;	//measure the battery voltage, convert to mV, put in array
	temp[dp] = rawADC[VTEMP] / (float)0xFFFF * 3300;	//measure the battery temp voltage, convert to mV, put in array
	temp[dp] = temp[dp] / 10 - 50;     			//convert to celsius, put in array

	//calculate battery current depending on mode
	if (state==CHG_MODE)			//if in charge mode
		perCurrent = rawADC[CHG_CURR] / (float)0xFFFF;
	else if (state==DIS_MODE)	//else if in discharge mode
		perCurrent = rawADC[DIS_CURR] / (float)0xFFFF;

	current[dp]	= perCurrent * maxCurrSense;	//convert percent current to mA and put in array

	//send error if current is too high
	if (perCurrent > PID_perCurrentLimit)   //make sure current is reading below 7A, 7A*.003ohm*100/3.3V = 0.64
	{
	    state = CYCLE_DONE;			//set state to cycle done
		doneReason = OVER_CURRENT;  //set error type to over current
		Stop_CHG_DIS(); //run function that turns off the charge or discharge
	}

	//calculate capacity
	capacity = capacity + ( current[dp] * (periodSec / 3600) );	// mAh + A/(1000mA/A) * (s / (60s/m) / (60m/h) )
	intCapacity = (int) capacity;		//convert to integer for display and storage
}

static void Chk_Complete(void){

	if (temp[dp] > OTthresh){        //if measured temperature exceeded temperature threshold
		state = CYCLE_DONE;             //set state to 'cycle done'
		doneReason = OVER_TEMP;			//set done reason

	}else if (state==CHG_MODE){              //if in charge state
		//determine maximum voltage so far
		if (VBat[dp] > maxVBat)     //if most recent voltage reading is greater than max reading so far
			maxVBat = VBat[dp];     //assign maxVBat new higher value

		//delta peak check
		deltaVBat = maxVBat - VBat[dp]; //calculate delta
		if (deltaVBat > deltaPeak){
			state = CYCLE_DONE;				//set state to 'cycle done'
			doneReason = DELTA_PEAK;		//set doneReason
		//if the charge timeout is reached
		}else if ((timeStamp[dp]*interval) >= (chgTimeout*60*interval)){
			state = CYCLE_DONE;					//set state to 'cycle done'
			doneReason = TIMEOUT;				//set doneReason
		}else if (intCapacity > chgCapLimit){//if the capacity limit is reached
			state = CYCLE_DONE;					//set sstate to 'cycle done'
			doneReason = OVER_CAPACITY;			//set doneReason
		}

	}else if (state==DIS_MODE){              //if in discharge state

		//check for cell dropout
		if (VBat[dp] < 7200)		//start checking when VBat is less than 7.2V
		{
			rateVBat = (VBat[dp] - preVBat)/periodSec; //calculate the instantaneous rate of Vbat change
			if (rateVBat < VBatRateLimit){//if the battery voltage rate is less than the cellDropRate
				state = CYCLE_DONE;
				doneReason = DROPOUT;
			}
			preVBat = VBat[dp];
		//else if battery voltage less than discharge voltage threshold
		}else if (VBat[dp] < disVcut){//if the latest measured voltage is less than the discharge cut off
			state = VBAT_CUT;			//set state to 'cycle done'
			doneReason = DROPOUT;		//set doneReason
		//else if the discharge timeout is reached
		}else if ((timeStamp[dp]*interval) >= (disTimeout*60*interval)){
			state = CYCLE_DONE;			//set state to 'cycle done'
			doneReason = TIMEOUT;		//set doneReason
		//else if the capacity limit is reached
		}else if (intCapacity > disCapLimit){
			state = CYCLE_DONE;			//set sstate to 'cycle done'
			doneReason = OVER_CAPACITY;	//set doneReason
		}
	}
}


//PID function to
static void Iterate_PID(void){
	float PID_error;		//expected output minus actual output
	float PID_output;		//Kp * err + (Ki * intError * dt) + (Kd * difError /dt);
	uint16_t int_PID_out;	//stores the output as an integer

	if (state<=1) {		//if cycle done
		Stop_CHG_DIS();		//stop current from flowing
	}else				//else in charge mode or discharge mode

	{
		PID_error = PID_perCurrentSet - perCurrent; 				//calculate PID error
		PID_intError = PID_intError + (PID_error * periodSec);		//calculate integral error
		PID_output = PID_Kp * PID_error + PID_Ki * PID_intError;	//PWM output value

		//if PID is trying to set current to < 0 value, set output to zero
		if (PID_output < 0)
		{
			PID_output = 0;
		}
		else if (PID_output > 1)
		{
			PID_output =1;
		}

		int_PID_out = PID_output * 0xFFFF;			//convert from float to 16 bit integer value
		int_PID_out = 0xFFFF - int_PID_out;			//PWMs are active low, subtract from full scale so PWM acts active high

		/* used for debugging*/
		//accumulator = accumulator - 100;
		//int_PID_out = accumulator;
		//int_PID_out = 65350;
		//printf("\n\r output=%f, error=%f, intError=%f",
		//int_PID_out, PID_output, PID_error, PID_intError);
		//if(timeStamp[dp] == 5)
		//{
			//set battery current depending on mode
			if (state==CHG_MODE)			//if in charge mode
				CHG_PWM_SetRatio16(int_PID_out);//set charge current
			else if (state==DIS_MODE)	//else if in discharge mode
				DIS_PWM_SetRatio16(int_PID_out);//set discharge current
		//}
	}
}

static void Log_Error(void){//error function used by LogToFile
	state = CYCLE_DONE;
	doneReason = LOG_ERROR;
}

static void Log_To_File(){	//log data to file on uSD
  uint8_t write_buf[48];
  UINT bw;
  TIMEREC time;

  /* open file */
  if (FAT1_open(&fp, "./log.txt", FA_OPEN_ALWAYS|FA_WRITE)!=FR_OK) {
    Log_Error();
  }
  /* move to the end of the file */
  if (FAT1_lseek(&fp, fp.fsize) != FR_OK || fp.fptr != fp.fsize) {
    Log_Error();
  }
  /* get time */
  if (TmDt1_GetTime(&time)!=ERR_OK) {
    Log_Error();
  }
  /* write data */
  write_buf[0] = '\0';
  UTIL1_strcatNum8u(write_buf, sizeof(write_buf), time.Hour);
  UTIL1_chcat(write_buf, sizeof(write_buf), ':');
  UTIL1_strcatNum8u(write_buf, sizeof(write_buf), time.Min);
  UTIL1_chcat(write_buf, sizeof(write_buf), ':');
  UTIL1_strcatNum8u(write_buf, sizeof(write_buf), time.Sec);
  UTIL1_chcat(write_buf, sizeof(write_buf), '\t');

  UTIL1_strcatNum32s(write_buf, sizeof(write_buf), timeStamp[dp]);
  UTIL1_chcat(write_buf, sizeof(write_buf), '\t');
  UTIL1_strcatNum32s(write_buf, sizeof(write_buf), current[dp]);
  UTIL1_chcat(write_buf, sizeof(write_buf), '\t');
  UTIL1_strcatNum32s(write_buf, sizeof(write_buf), VBat[dp]);
  UTIL1_chcat(write_buf, sizeof(write_buf), '\t');
  UTIL1_strcatNum32s(write_buf, sizeof(write_buf), intCapacity);
  UTIL1_chcat(write_buf, sizeof(write_buf), '\t');
  UTIL1_strcatNum32s(write_buf, sizeof(write_buf), temp[dp]);
  UTIL1_strcat(write_buf, sizeof(write_buf), (unsigned char*)"\r\n");
  if (FAT1_write(&fp, write_buf, UTIL1_strlen((char*)write_buf), &bw)!=FR_OK) {
    (void)FAT1_close(&fp);
    Log_Error();
  }
  /* closing file */
  (void)FAT1_close(&fp);
}

void APP_Run(void) {
  SHELL_Init();
  /*if (FRTOS1_xTaskCreate(
        R_LEDblink,  // pointer to the task
        "Task1", // task name for kernel awareness debugging
        configMINIMAL_STACK_SIZE, // task stack size
        (void*)NULL, // optional task startup argument
        tskIDLE_PRIORITY,  // initial priority
        (xTaskHandle*)NULL // optional task handle to create
      ) != pdPASS) {
    //lint -e527
    for(;;){}; // error! probably out of memory
    //lint +e527
  }*/
  if (FRTOS1_xTaskCreate(
          control,  /* pointer to the task */
          "main", /* task name for kernel awareness debugging */
          configMINIMAL_STACK_SIZE+200, /* task stack size */
          (void*)NULL, /* optional task startup argument */
          tskIDLE_PRIORITY,  /* initial priority */
          (xTaskHandle*)NULL /* optional task handle to create */
        ) != pdPASS) {
      /*lint -e527 */
      for(;;){}; /* error! probably out of memory */
      /*lint +e527 */
    }
  FRTOS1_vTaskStartScheduler();
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
byte BTS_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io)
{
  if (UTIL1_strcmp((char*)cmd, "BTS help")==0) {
    *handled = TRUE;
    return PrintHelp(io);
  } else if (UTIL1_strcmp((char*)cmd, "BTS status")==0) {
    *handled = TRUE;
    return PrintStatus(io);
  } else if (UTIL1_strncmp((char*)cmd, "BTS chgi ", sizeof("BTS chgi ")-1)==0) {
	uint8_t res;
	const unsigned char *p;
	p = cmd+sizeof("BTS chgi ")-1;
	res = UTIL1_ScanDecimal32sNumber(&p, &chgCurrSet);
	if (res!=ERR_OK) {
	  CLS1_SendStr((unsigned char*)"*** invalid number format! Try e.g. 10\r\n", io->stdErr);
	  return ERR_FAILED;
	}
	else{
		Load_Current(chgCurrSet);
	}
	*handled = TRUE;
	return ERR_OK;
  } else if (UTIL1_strncmp((char*)cmd, "BTS disi ", sizeof("BTS disi ")-1)==0) {
		uint8_t res;
		const unsigned char *p;
		p = cmd+sizeof("BTS disi ")-1;
		res = UTIL1_ScanDecimal32sNumber(&p, &disCurrSet);
		if (res!=ERR_OK) {
		  CLS1_SendStr((unsigned char*)"*** invalid number format! Try e.g. 10\r\n", io->stdErr);
		  return ERR_FAILED;
		}
		else{
				Load_Current(disCurrSet);
		}
		*handled = TRUE;
		return ERR_OK;
  }else if (UTIL1_strcmp((char*)cmd, "BTS start")==0) {
	    *handled = TRUE;
	    return Start(io);
  }else if (UTIL1_strcmp((char*)cmd, "BTS stop")==0) {
	    *handled = TRUE;
	    return Stop(io);
  }else if (UTIL1_strcmp((char*)cmd, "BTS verbose")==0) {
	    *handled = TRUE;
	    return Toggle_Verbose(io);
  }else if (UTIL1_strcmp((char*)cmd, "BTS logging")==0) {
  	    *handled = TRUE;
  	    return Toggle_Logging(io);
  }else if (UTIL1_strncmp((char*)cmd, "BTS mode ", sizeof("BTS mode ")-1)==0) {
		uint8_t res;
		const unsigned char *p;
		p = cmd+sizeof("BTS mode ")-1;
		res = UTIL1_ScanDecimal32sNumber(&p, &firstState);
		if (res!=ERR_OK) {
		  CLS1_SendStr((unsigned char*)"*** invalid number format! Try e.g. 10\r\n", io->stdErr);
		  return ERR_FAILED;}
		if (firstState == CHG_MODE){
			CLS1_SendStr((unsigned char*)"begin with charge\r\n", io->stdErr);
		}else if(firstState == DIS_MODE){
			CLS1_SendStr((unsigned char*)"begin with discharge\r\n", io->stdErr);
		}else{
			CLS1_SendStr((unsigned char*)"*** only valid options are 2 or 3\r\n", io->stdErr);
			firstState = CHG_MODE;
			CLS1_SendStr((unsigned char*)"first state set to charge mode\r\n", io->stdErr);
		}
		*handled = TRUE;
		return ERR_OK;
  } /* else if (UTIL1_strncmp((char*)cmd, "FAT1 delete", sizeof("FAT1 delete")-1)==0) {
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
  }*/
  return ERR_OK;
}


//used by BTS_ParseCommand to print help message
static uint8_t PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"BTS", (unsigned char*)"Group of BTS commands\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Print help or status information\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  chgi <current>", (const unsigned char*)"Set the charge current in mA\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  disi <current>", (const unsigned char*)"Set the discharge current in mA\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  serial <pack serial>", (const unsigned char*)"Set pack serial\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  subpack <subpack code>", (const unsigned char*)"set subpack code\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  start", (const unsigned char*)"start battery capacity test\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  stop", (const unsigned char*)"stop test and save data\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  break", (const unsigned char*)"stop test and discard data\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  verbose", (const unsigned char*)"toggle print log to shell\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  logging", (const unsigned char*)"toggle log data to uSD\r\n", io->stdOut);
  return ERR_OK;
}

//used by BTS_ParseCommand to print status message
static uint8_t PrintStatus(const CLS1_StdIOType *io) {

	//int32_t duration = timeStamp[dp];
	//CLS1_SendStatusStr((unsigned char*)"BTS", (unsigned char*)"\r\n", io->stdOut);
	UTIL1_Num32sToStr(charBuff,sizeof(charBuff),state);
	CLS1_SendStatusStr((unsigned char*)"  mode", charBuff, io->stdOut);
	UTIL1_Num32sToStr(charBuff,sizeof(charBuff),timeStamp[dp]);
	CLS1_SendStatusStr((unsigned char*)"  Loops", charBuff, io->stdOut);
	UTIL1_Num32sToStr(charBuff,sizeof(charBuff),current[dp]);
	CLS1_SendStatusStr((unsigned char*)"  Ibat[mA]", charBuff, io->stdOut);
	UTIL1_Num32sToStr(charBuff,sizeof(charBuff),VBat[dp]);
	CLS1_SendStatusStr((unsigned char*)"  Vbat[mV]", charBuff, io->stdOut);
	UTIL1_Num32sToStr(charBuff,sizeof(charBuff),intCapacity);
	CLS1_SendStatusStr((unsigned char*)"  cap[mAh]", charBuff, io->stdOut);
	UTIL1_Num32sToStr(charBuff,sizeof(charBuff),temp[dp]);
	CLS1_SendStatusStr((unsigned char*)"  batTemp[C]", charBuff, io->stdOut);
	CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
	return ERR_OK;
}

//loads the current
static void Load_Current(int newCurrent){
    /*scale the current setting to a percentage(0 to 1), current sense amplifier is measureing voltage across a .003ohm resisitor with a gain of 100
        Vref is 3.3V, therefore the maximum current that can be read is 3.3V/(.003ohm*100) = 11Amps*/
	PID_perCurrentSet = (float)newCurrent / maxCurrSense;	//newCurrent in mA divided by the maximum current the current sense adc channels can read
}

static void State_Machine(const CLS1_StdIOType *io){
	unsigned int waitTimeMs;

	if (doneReason == DELTA_PEAK){
		nextState = DIS_MODE;
		CLS1_SendStr((unsigned char*)"delta peak V detected, charge cycle complete", io->stdOut);	//print status message
	}else if (doneReason == VBAT_CUT){
		nextState = CHG_MODE;
		CLS1_SendStr((unsigned char*)"battery V cut off reached, discharge cycle complete", io->stdOut);	//print status message
	}else if (doneReason == DROPOUT){
		nextState = CHG_MODE;
		CLS1_SendStr((unsigned char*)"cell dropout occured, discharge cycle complete", io->stdOut);	//print status message
	}else if (doneReason == TIMEOUT){
		CLS1_SendStr((unsigned char*)"cycle ended due to timeout\r\n", io->stdOut);
		nextState = IDLE_MODE;
	}else if (doneReason == OVER_TEMP){
		CLS1_SendStr((unsigned char*)"cycle ended due to over temperature\r\n", io->stdOut);
		nextState = IDLE_MODE;
	}else if (doneReason == USER_COMMAND){
		CLS1_SendStr((unsigned char*)"cycle ended due to user command\r\n", io->stdOut);
		nextState = IDLE_MODE;
	}else if (doneReason == OVER_CURRENT){
		CLS1_SendStr((unsigned char*)"cycle ended due to over-current condition\r\n", io->stdOut);
		nextState = IDLE_MODE;
	}else if (doneReason == LOG_ERROR){
		CLS1_SendStr((unsigned char*)"cycle ended due to log error\r\n", io->stdOut);
		nextState = IDLE_MODE;
	}

	if (doneReason <= DROPOUT){
		waitTimeMs = waitTime * 60000;	//calculate wait time in ms
		UTIL1_Num32uToStr(charBuff,64,waitTime);
		CLS1_SendStatusStr((unsigned char*)" waiting[min]", charBuff, io->stdOut);
		FRTOS1_vTaskDelay(waitTimeMs/portTICK_RATE_MS);	//wait between cycles
	}

	doneReason = 0;		//reset done reason
}

//command for starting charge/discharge program
static uint8_t Start(const CLS1_StdIOType *io){

	if (state == IDLE_MODE){	//if in idle mode
		state = CYCLE_DONE;			//set state to cycle done, used as flag to run Load_Settings function
		nextState = firstState;		//initialize nextState, Load_Settings will load this into state
		cyclesLeft = maxCycles;		//load cyclesLeft with maxCycles
		CLS1_SendStr((unsigned char*)"BTS starting\r\n", io->stdOut);

	}else
		CLS1_SendStr((unsigned char*)"already started\r\n", io->stdOut);

	return ERR_OK;
}

//command for stopping
static uint8_t Stop(const CLS1_StdIOType *io){

	if (state >= INT_RES_TEST){
		state = CYCLE_DONE;
		doneReason = USER_COMMAND;
	}else
		CLS1_SendStr((unsigned char*)"already stopped\r\n", io->stdOut);

	return ERR_OK;
}

//command for toggling verbose mode
static uint8_t Toggle_Verbose(const CLS1_StdIOType *io){

	if (verbose == 1){
		verbose = 0;
		CLS1_SendStr((unsigned char*)"verbose disabled\r\n", io->stdOut);
	}

	else{
		verbose = 1;
		CLS1_SendStr((unsigned char*)"verbose enabled\r\n", io->stdOut);
	}

	return ERR_OK;
}

//command for stopping
static uint8_t Toggle_Logging(const CLS1_StdIOType *io){

	if (logEnabled == 1){
		logEnabled = 0;
		CLS1_SendStr((unsigned char*)"logging disabled\r\n", io->stdOut);
	}

	else{
		logEnabled = 1;
		CLS1_SendStr((unsigned char*)"logging enabled\r\n", io->stdOut);
	}

	return ERR_OK;
}
