/*
 * Application.c
 *      Author: Erich Styger
 */
#include "Application.h"
#include "LEDR.h"
#include "LEDG.h"
#include "FRTOS1.h"
#include "Shell.h"
#include "CHG_EN.h"
#include "CHG_PWM.h"
#include "DIS_PWM.h"
#include "AD1.h"
#include "WAIT1.h"
#include <stdio.h>

/* Global Variales */
static int state=chgMode;  //used to control the run mode of the system idleMode, intResTest, chgMode, disMode, cycleDone
static int errorType=0;            //declare int used to indicate the error type, 0=no error, 1=current too high
static float currentSet=1;         //declare float, sets the battery current, in amps, don't set higher than 7
static float chgCurrSet=0;         //charge current in A
static float disCurrSet=0;         //discharge current in A
static float currentLimit=6.5;     //current limit in A
static float deltaPeak=0.024;      //delta peak charge cutoff voltage in V (6 cells * 4mV)
static float disVcut=5.4;          //discharge voltage cutoff in V
static float cellDropRate = -.01;  //cell dropout rate [V/s]
static float chgTimeout = 120;     //charge timeout in minutes
static float disTimeout = 120;     //discharge timeout in minutes
static float OTthresh = 50;        //over temperature threshhold in celsius
static int numCycles = 1;          //number of charge/discharge cycles
static int startFirst = 0;         //0=charge first, 1=discharge first
static float perCurrent;           //float, stores the battery current read by the ADC, between 0 and 1
static float current[4];          //float array, the battery current in Amps, saved to usd card
static float VBat[4];             //float array, stores the battery voltage in Volts, saved to usd card
static float temp[4];             //float array, stores the temperature sensor reading in Volts
static int timeStamp[4];          //int array, stores the time a measurement was made
static int dp=0;                   //data pointer, used to indicate which element is most recent in the measured data arrays
static int measure_time;           //declare int, stores the current measurement iteration for the entire charge or discharge cycle
static float measure_period=1;   //period between ADC measurements in seconds


static portTASK_FUNCTION(R_LEDblink, pvParameters) {
  (void)pvParameters; /* parameter not used */
  for(;;) {
    LEDR_Neg();
    FRTOS1_vTaskDelay(5000/portTICK_RATE_MS);
  }
}

static portTASK_FUNCTION(control, pvParameters) {
  (void)pvParameters; /* parameter not used */
  for(;;) {

	  //if this is the first loop load settings
	  if (timeStamp[0] = -1)
		loadSettings();

	  while(state >= chgMode){

		  //measure
		  //check if done
		  iteratePID();
		  //write data
		  LEDG_Neg();
		  FRTOS1_vTaskDelay(1000/portTICK_RATE_MS);
	  }


    //check shell for command

	  FRTOS1_vTaskDelay(1000/portTICK_RATE_MS);
  }
}

//charge enable=1
//set output
//setup (load values, reset PID etc)
//loop 1
//read adcs
//chk if done
//update PID
//update PWM duty
//write to file system
//


//PID variables
static float PID_perCurrentSet;        	//float, stores the battery current setting scaled between 0 and 1
static float PID_perCurrentLimit;      	//float, stores current limit scaled between 0 and 1
static float PID_Kp = 0.1;             	//Proportional constant
static float PID_Ki = 0.01;             	//Integral constant
//float Kd = 0.1;             //Derivative constant
static float PID_error;               	//expected output minus actual output
//float prevError = 0;        //previous error value
static float PID_intError = 0;       	//integration result from previous loop + error * PID_period
//float difError;             //Differential error, difError - err from previous loop'
static float PID_period = 1;			//execution time of loop in seconds
static float PID_output ;				//Kp * err + (Ki * intError * dt) + (Kd * difError /dt);

//PID function to
static void iteratePID(void){
	uint16_t int_PID_out;	//stores the output as an integer
	uint16_t adcResults[AD1_CHANNEL_COUNT];	//stores latest adc measurements

	if (state<=1) {}                    //if off or measuring resistance, do nothing
		else                                //else in charge mode or discharge mode
		{

			AD1_Measure(1);		//start adc conversion and wait till complete
			AD1_GetValue16(&adcResults[0]);

			//send error if current is too high
			if (perCurrent > PID_perCurrentLimit)   //make sure current is reading below 7A, 7A*.003ohm*100/3.3V = 0.64
			{
				state = -1;     //set state to done
				errorType = 1;  //set error type to 'current too high'
				stop_CHG_DIS(); //run function that turns off the charge or discharge
			}

			PID_error = PID_perCurrentSet - perCurrent; 				//calculate PID error
			PID_intError = PID_intError + (PID_error * PID_period);		//calculate integral error
			PID_output = PID_Kp * PID_error + PID_Ki * PID_intError;	//PWM output value
			int_PID_out = PID_output * 0xFFFF;							//convert from float to 16 bit integer value

			/* used for debugging*/
			printf("\n\r output=%f, error=%f, intError=%f",
			int_PID_out, PID_output, PID_error, PID_intError);

			//set battery current depending on mode
			if (state==chgMode)			//if in charge mode
				CHG_PWM_SetRatio16(int_PID_out);//set charge current
			else if (state==disMode)	//else if in discharge mode
				DIS_PWM_SetRatio16(int_PID_out);//set discharge current
		}
}

static void loadSettings(void){
    /*scale the current setting to a percentage(0 to 1), current sense amplifier is measureing voltage across a .003ohm resisitor with a gain of 100
        Vref is 3.3V, therefore the maximum current that can be read is 3.3V/(.003ohm*100) = 11Amps*/
    PID_perCurrentSet = currentSet / 11;    //currentSet in amps divided by 11A, the maximum current the current sense adc channels can read
    PID_perCurrentLimit = currentLimit / 11;//
}

//function to stop charging or discharging
void stop_CHG_DIS(void)
{
    CHG_EN_ClrVal();        	//turn off PFETs, if not already off
    CHG_PWM_SetRatio16(0xFFFF);	//turn on CHG NFET which connects the negative side of the battery to PGND
    DIS_PWM_SetRatio16(0);      //turn off discharge NFET if not already off to disconect the load from the circuit.
    WAIT1_Waitms(1000);        	//wait 1 second to allow capacitors to discharge
    CHG_PWM_SetRatio16(0);		//turn off CHG NFET to completely disconect battery from circuit
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
