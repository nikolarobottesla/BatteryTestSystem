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
#include <stdio.h>

/* Global Variales */
static int state=chgMode;			//used to control the run mode of the system idleMode, intResTest, chgMode, disMode, cycleDone
static int numCycles = 1;			//number of charge/discharge cycles
static int startFirst = 0;			//0=charge first, 1=discharge first
static int errorType=0;				//declare int used to indicate the error type, 0=no error, 1=current too high
static int doneReason;				//reason the cycle ended, 1= delta peak, 2= temp limit, 3= time limit, 4= discharge Vcut, 5=cell dropout
static float currentSet=0;		//declare float, sets the battery current, in amps, don't set higher than 7
static float chgCurrSet=0;			//charge current in A
static float disCurrSet=0;			//discharge current in A
static float currentLimit=6.5;		//current limit in A
static float maxCurrSense=11;		//the maximum current the adc can sense
static float deltaPeak=0.024;		//delta peak charge cutoff voltage in V (6 cells * 4mV)
static float disVcut=5.4;			//discharge voltage cutoff in V
static float VBatRateLimit = -.01;	//cell dropout rate [V/s]
static float chgTimeout = 120;		//charge timeout in minutes
static float disTimeout = 120;		//discharge timeout in minutes
static TickType_t interval = 1000;	//measurement interval in ms
//static float chgCapLimit = 12000;	//charge capacity limit [mAh]
//static float disCapLimit = 8000;	//discharge capacity limit [mAh]
//static float capacity;				//current measured capacity
static float OTthresh = 50;        	//over temperature threshhold in celsius
//PID variables, PID controls /charge or discharge current
static float PID_Kp = 0.1;             	//Proportional constant
static float PID_Ki = 0.05;             //Integral constant
//float Kd = 0.1;             //Derivative constant
//float prevError = 0;        //previous error value
//float difError;             //Differential error, difError - err from previous loop'
static float perCurrent;           	//float, stores the battery current read by the ADC, between 0 and 1
static float current[dataBuffSize];	//float array, the battery current in Amps, saved to usd card
static float VBat[dataBuffSize];	//float array, stores the battery voltage in Volts, saved to usd card
static float temp[dataBuffSize];	//float array, stores the temperature sensor reading in Volts
static int timeStamp[dataBuffSize]={-1};	//stores the time a measurement was made,
static int lastStamp;				//stores last loop's timestamp
static int dp=0;					//data pointer, used to indicate which element is most recent in the measured data arrays
static int measure_time;			//stores the current measurement iteration for the entire charge or discharge cycle
static float periodSec;				//period between ADC measurements in seconds
static uint16_t rawADC[AD1_CHANNEL_COUNT];	//stores latest adc measurements
static float maxVBat =0;			//maximum battery voltage, Volts
static float deltaVBat;				//difference between the maximum and most recently measured battery voltage, Volts
static float preVBat;				//stores the previous Vbat measurement
static float rateVBat;				//the battery voltage rate, used for cell dropout check [volts/sec]
static float PID_perCurrentSet;		//float, stores the battery current setting scaled between 0 and 1
static float PID_perCurrentLimit;	//float, stores current limit scaled between 0 and 1
static float PID_intError;			//integration result from previous loop + error * periodSec
static unsigned char buffer[64];	//used to store
static unsigned int accumulator = 65535;	//used for debugging

static portTASK_FUNCTION(R_LEDblink, pvParameters) {
  (void)pvParameters; /* parameter not used */

  buffer[0] = '\0'; /* initialize buffer for ReadLine() */
  for(;;) {
	CLS1_SendStr("Type in some text with CR or LF at the end...\r\n", CLS1_GetStdio()->stdOut);
	if (CLS1_ReadLine(buffer, buffer, sizeof(buffer), CLS1_GetStdio())) {
	  /* line read */
	  CLS1_SendStr("You entered:\r\n", CLS1_GetStdio()->stdOut);
	  CLS1_SendStr(buffer, CLS1_GetStdio()->stdOut);
	  buffer[0] = '\0';
	}
    //LEDR_Neg();
    FRTOS1_vTaskDelay(5000/portTICK_RATE_MS);
  }
}

static portTASK_FUNCTION(control, pvParameters) {
  (void)pvParameters; /* parameter not used */

  //initialization
  CHG_PWM_Enable();
  DIS_PWM_Enable();
  stop_CHG_DIS();
  AD1_Calibrate(1);	//calibrate adc and wait till done
  TickType_t xLastWakeTime= xTaskGetTickCount(); //Declare and init the xLastWakeTime variable with the current time.

  for(;;) {

	  //if this is the first loop load settings
	  if (timeStamp[0] == -1){
		  loadSettings();	//load settings, eventually want to load from a file on SD card
		  timeStamp[0] = 0;	//set initial timeStamp
	  }

	  state = chgMode;	//for debugging

	  while(state >= chgMode){

		  measureAll();
		  //chkComplete();

		  iteratePID();
		  //write data
		  LEDR_Neg();
		  lastStamp = timeStamp[dp];	//save the last time stamp for use in the next loop iteration

		  dp++;							//increment pointer for data buffers
		  if (dp >= dataBuffSize)
		  {
			  dp=0;						//reset pointer for data buffers
		  }

		  //CLS1_SendStr("Type in some text with CR or LF at the end...\r\n", CLS1_GetStdio()->stdOut);

		  FRTOS1_vTaskDelayUntil(&xLastWakeTime, interval/portTICK_RATE_MS);	//using this vs TaskDelay to get a specific period
	  }

	  FRTOS1_vTaskDelay(5000/portTICK_RATE_MS);
  }
}

//function to stop charging or discharging, PWMs are active low
void stop_CHG_DIS(void)
{
    CHG_PWM_SetRatio16(0xFFFF);	//turn off CHG PFETs to disconnect 12V
    DIS_PWM_SetRatio16(0xFFFF); //turn off discharge NFET if not already off to disconect the load from the circuit.
	state = -1;     			//set state to done
}

static void loadSettings(void){
    /*scale the current setting to a percentage(0 to 1), current sense amplifier is measureing voltage across a .003ohm resisitor with a gain of 100
        Vref is 3.3V, therefore the maximum current that can be read is 3.3V/(.003ohm*100) = 11Amps*/
    PID_perCurrentSet = currentSet / maxCurrSense;    //currentSet in amps divided by 11A, the maximum current the current sense adc channels can read
    PID_perCurrentLimit = currentLimit / maxCurrSense;//
    preVBat=0;								//initialized at 0V to prevent cell dropout detector from triggering on 1st check
    periodSec = interval / 1000;		//calculate the measure period in seconds
    PID_intError = 0;
}

static void measureAll(void){
	AD1_Measure(1);					//start adc conversion and wait till complete
	AD1_GetValue16(&rawADC[0]);		//put ADC values in array rawADC
	VBat[dp] = rawADC[VBAT_SE] / 0xFFFF * 9.9;	//measure the battery voltage, convert to volts, put in array
	temp[dp] = rawADC[VTEMP] / 0xFFFF *3.3;		//measure the battery temp, convert to volts, put in array
	temp[dp] = temp[dp]/.01-50;     			//convert to celsius, put in array
	timeStamp[dp] = lastStamp + 1;

	 /* used for debugging*/
	//pc.printf("\n\r dp = %d, current=%f, VBat=%f, temp=%f"
	//		,dp , current[dp],VBat[dp],temp[dp]);
}

static void chkComplete(void){

	if (temp[dp] > OTthresh)        //if measured temperature exceeded temperature threshold
		state = -1;                     //set state to 'cycle done'
	else if (state==2)              //if in charge state
	{

		//if the charge timeout is reached
		if ((timeStamp[dp]*interval) >= (chgTimeout*60*interval))
			state = -1;		//set state to 'cycle done'

		//determine maximum voltage so far
		if (VBat[dp] > maxVBat)         //if most recent voltage reading is greater than max reading so far
			maxVBat = VBat[dp];             //assign maxVBat new higher value

		//delta peak check
		deltaVBat = maxVBat - VBat[dp]; //calculate delta
		if (deltaVBat > deltaPeak)      //check if delta is larger than allowed
			state = -1;                     //set state to 'cycle done'

		//capacity check

	}
	else if (state==3)              //if in discharge state
	{
		//if the discharge timeout is reached
		if ((timeStamp[dp]*interval) >= (disTimeout*60*interval))
			state = -1;		//set state to 'cycle done'

		//measure threshhold
		if (VBat[dp] < disVcut)         //if the latest measured voltage is less than the discharge cut off
			state = -1;                     //set state to 'cycle done'

		//check for cell dropout
		if (VBat[dp] < 7.2)             //start checking when VBat is less than 7.2V
		{
			rateVBat = (VBat[dp] - preVBat)/periodSec; //calculate the instantaneous rate of Vbat change
			if (rateVBat < VBatRateLimit){           		//if the battery voltage rate is less than the percent cellDropRate
				state = -1;
				doneReason = 5;
			}
			preVBat = VBat[dp];
		}
	}
}


//PID function to
static void iteratePID(void){
	float PID_error;		//expected output minus actual output
	float PID_output;		//Kp * err + (Ki * intError * dt) + (Kd * difError /dt);
	uint16_t int_PID_out;	//stores the output as an integer

	if (state<=1) {}                    //if off or measuring resistance, do nothing
		else                                //else in charge mode or discharge mode
		{

			//read battery current depending on mode
			if (state==chgMode)			//if in charge mode
				perCurrent = rawADC[CHG_CURR] / (float)0xFFFF;
			else if (state==disMode)	//else if in discharge mode
				perCurrent = rawADC[DIS_CURR] / (float)0xFFFF;

			current[dp]	= perCurrent * maxCurrSense;	//convert percent current to A and put in array

			//send error if current is too high
			if (perCurrent > PID_perCurrentLimit)   //make sure current is reading below 7A, 7A*.003ohm*100/3.3V = 0.64
			{
				errorType = 1;  //set error type to 'current too high'
				stop_CHG_DIS(); //run function that turns off the charge or discharge
			}

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
				if (state==chgMode)			//if in charge mode
					CHG_PWM_SetRatio16(int_PID_out);//set charge current
				else if (state==disMode)	//else if in discharge mode
					DIS_PWM_SetRatio16(int_PID_out);//set discharge current
			//}
		}
}

void APP_Run(void) {
  SHELL_Init();
  if (FRTOS1_xTaskCreate(
        R_LEDblink,  /* pointer to the task */
        "Task1", /* task name for kernel awareness debugging */
        configMINIMAL_STACK_SIZE, /* task stack size */
        (void*)NULL, /* optional task startup argument */
        tskIDLE_PRIORITY,  /* initial priority */
        (xTaskHandle*)NULL /* optional task handle to create */
      ) != pdPASS) {
    /*lint -e527 */
    for(;;){}; /* error! probably out of memory */
    /*lint +e527 */
  }
  if (FRTOS1_xTaskCreate(
          control,  /* pointer to the task */
          "Task2", /* task name for kernel awareness debugging */
          configMINIMAL_STACK_SIZE, /* task stack size */
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
