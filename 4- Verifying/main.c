/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "message_buffer.h"
#include "semphr.h"


/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )




/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/
TaskHandle_t Button_1_Monitor_Handler = NULL;
TaskHandle_t Button_2_Monitor_Handler = NULL;
TaskHandle_t Periodic_Transmitter_Handler = NULL;
TaskHandle_t Uart_Receiver_Handler = NULL;
TaskHandle_t Load_1_Simulation_Handler = NULL;
TaskHandle_t Load_2_Simulation_Handler = NULL;

SemaphoreHandle_t xSemaphore_BTN1;
SemaphoreHandle_t xSemaphore_BTN2;

int Task_1_inTime=0,Task_1_outTime=0,Task_1_TotTime=0;
int Task_2_inTime=0,Task_2_outTime=0,Task_2_TotTime=0;
int Task_3_inTime=0,Task_3_outTime=0,Task_3_TotTime=0;
int Task_4_inTime=0,Task_4_outTime=0,Task_4_TotTime=0;
int Task_5_inTime=0,Task_5_outTime=0,Task_5_TotTime=0;
int Task_6_inTime=0,Task_6_outTime=0,Task_6_TotTime=0;

int Sys_Time=0;
float CPU_Load=0;

pinState_t Button_1_State;
pinState_t Button_2_State;

char RunTimeBuff[200];
QueueHandle_t Q1 = NULL;
QueueHandle_t Q2 = NULL;
QueueHandle_t Q3 = NULL;

void Button_1_Monitor( void * pvParameters )
{
  pinState_t Button1Status;
	pinState_t lastStatus = GPIO_read(PORT_1 , PIN0);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	signed char Edge = 0;
vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	for( ;; )
	{
		/* Read GPIO Input */
		Button1Status = GPIO_read(PORT_1 , PIN0);
		
		/*Check for Edges*/
		if( Button1Status == PIN_IS_HIGH && lastStatus == PIN_IS_LOW)
		{
			/*Positive Edge*/
			Edge = '+';
		}
		else if (Button1Status == PIN_IS_LOW && lastStatus == PIN_IS_HIGH)
		{
			/*Negative Edge*/
			Edge = '-';
		}
		else
		{
			Edge = '.';
		}
		/*Update Button State*/
		lastStatus = Button1Status;
		
		/*Send Data to consumer*/
		xQueueOverwrite( Q1 , &Edge );

		/*Periodicity: 50*/
		vTaskDelayUntil( &xLastWakeTime , 50);
	}
}

void Button_2_Monitor( void * pvParameters )
{
    pinState_t lastState = GPIO_read(PORT_1 , PIN0);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	signed char Edge = 0;
	pinState_t Button2Status;
 vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
	for( ;; )
	{
		/* Read GPIO Input */
		Button2Status = GPIO_read(PORT_1 , PIN1);
		/*Check for Edges*/
		if( Button2Status == PIN_IS_HIGH && lastState == PIN_IS_LOW)
		{
			/*Positive Edge*/
			Edge = '+';
		}
		else if (Button2Status == PIN_IS_LOW && lastState == PIN_IS_HIGH)
		{
			/*Negative Edge*/
			Edge = '-';
		}
	
		else
		{
			Edge = '.';
		}
		/*Update Button State*/
		lastState = Button2Status;
		
		/*Send Data to consumer*/
		xQueueOverwrite( Q2 , &Edge );
 
		/*Periodicity: 50*/
		vTaskDelayUntil( &xLastWakeTime , 50);
	}
}

void Periodic_Transmitter( void * pvParameters )
{
   TickType_t xLastWakeTime = xTaskGetTickCount();
	uint8_t i = 0;
	char Str[13];
	strcpy(Str, "\nHello RTOS.");
  vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	for( ; ; )
	{
		/*Send string characters over Queue Q3 to Uart_Receiver*/
		for( i=0 ; i<13 ; i++)
		{
			xQueueSend( Q3 , Str+i ,100);
		}
		
		/*Periodicity: 100*/
		vTaskDelayUntil( &xLastWakeTime , 100);
	}
}

void Uart_Receiver( void * pvParameters )
{
   TickType_t xLastWakeTime = xTaskGetTickCount();
	signed char B1;
	signed char B2;
	char rxString[13];
	uint8_t i=0;
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	for( ; ; )
	{
		/*Receive Button 1 State*/
		if( xQueueReceive( Q1, &B1 , 0) && B1 != '.')
		{
			/*Transmit if +/- edge detected*/
			xSerialPutChar('\n');		
			xSerialPutChar('B');
			xSerialPutChar('1');
			xSerialPutChar(':');
			xSerialPutChar(B1);
		}
		else
		{
			/*Transmit spaces to take the same execution time 
			*even if Received nothing */
			xSerialPutChar(' ');
			xSerialPutChar(' ');
			xSerialPutChar(' ');
			xSerialPutChar(' ');
			xSerialPutChar(' ');
		}
		
		/*Receive Button 2 state*/
		if( xQueueReceive( Q2, &B2 , 0) && B2 != '.')
		{
			/*Transmit if +/- edge detected*/
			xSerialPutChar('\n');		
			xSerialPutChar('B');
			xSerialPutChar('2');
			xSerialPutChar(':');
			xSerialPutChar(B2);
		}
		else
		{
			/*Transmit spaces to take the same execution time 
			*even if Received nothing */
			xSerialPutChar(' ');		
			xSerialPutChar(' ');
			xSerialPutChar(' ');
			xSerialPutChar(' ');
			xSerialPutChar(' ');
		}
		
		/*Receive String from Periodic_Transmitter*/
		if( uxQueueMessagesWaiting(Q3) != 0)
		{
			for( i=0 ; i<15 ; i++)
			{
				xQueueReceive( Q3, (rxString+i) , 0);
			}
			vSerialPutString( (signed char *) rxString, strlen(rxString));
			xQueueReset(Q3);
		}
		

		
		
		/*Periodicity: 20*/
		vTaskDelayUntil( &xLastWakeTime , 20);
	}
}
#define ms 12000

void Load_1_Simulation ( void * pvParameters )
{
		TickType_t xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
		vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	
	for( ; ; )
	{
		uint32_t i = 0;
	uint32_t x = 12000*5; /* (XTAL / 1000U)*time_in_ms  */
		for( i=0 ; i <= x; i++)
		{
			/*5ms delay*/
		}
	
			vTaskDelayUntil( &xLastWakeTime, 10 );
    }
}

void Load_2_Simulation ( void * pvParameters )
{
		TickType_t xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
		vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	
	for( ; ; )
	{		
		 uint32_t i = 0;
	uint32_t x = 12000*12; /* (XTAL / 1000U)*time_in_ms  */
		for( i=0 ; i <= x; i++)
		{
			/*12ms delay*/
		}
			vTaskDelayUntil( &xLastWakeTime, 100 );
    }
}


void vApplicationTickHook (void){

	GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN0, PIN_IS_LOW);

}
void vApplicationIdleHook (void){

	GPIO_write(PORT_0, PIN7, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN7, PIN_IS_LOW);

}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	/*  Creating the message buffer */
	Q1 = xQueueCreate( 1, sizeof(char) );
  Q2 = xQueueCreate( 1, sizeof(char) );
  Q3 = xQueueCreate( 15, sizeof(char) );
	
    /* Create Tasks here */
	xTaskPeriodicCreate(Button_1_Monitor,"Button_1_Monitor",100,( void * ) 0,1,&Button_1_Monitor_Handler,50);
	xTaskPeriodicCreate(Button_2_Monitor,"Button_2_Monitor",100,( void * ) 0,1,&Button_2_Monitor_Handler,50);
	xTaskPeriodicCreate(Periodic_Transmitter,"Periodic_Transmitter",100,( void * ) 0,1,&Periodic_Transmitter_Handler,100);
	
	xTaskPeriodicCreate(Load_1_Simulation,"Load_1_Simulation",100,( void * ) 0,1,&Load_1_Simulation_Handler,10);
	xTaskPeriodicCreate(Load_2_Simulation,"Load_2_Simulation",100,( void * ) 0,1,&Load_2_Simulation_Handler,100);
	xTaskPeriodicCreate(Uart_Receiver,"Uart_Receiver",100,( void * ) 0,1,&Uart_Receiver_Handler,20);
		
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}
/* Function to create the buffer	*/


static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/

