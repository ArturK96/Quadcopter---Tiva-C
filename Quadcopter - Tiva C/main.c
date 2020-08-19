#include <stdio.h>
#include <stdint.h>
#include "math.h"
#include "mat.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "driverlib/sysctl.h"
#include "LED_LIB.h"
#include "ADC_LIB.h"
#include "PWM_LIB.h"
#include "Timer_LIB.h"
#include "UART_LIB.h"
#include "I2C_LIB.h"
#include "MPU6050_LIB.h"
#include "InterruptHandlers_LIB.h"
#include "PID_LIB.h"
#include "MOTOR_CONTROL_LIB.h"


uint32_t SystemCoreClock = 80000000;

TaskHandle_t myTask1Handle = NULL;
TaskHandle_t myTask2Handle = NULL;
QueueHandle_t myQueue;


uint16_t zmienna_1;
char str[42];


void myTask1(void *p){
	TickType_t myLastUnblock;
	myLastUnblock = xTaskGetTickCount();
	
	while(1){
		Read_MPU6050_Data();
		Calculate_MPU6050_Angles();
		
		Secure_Startup();
		Calculate_PID_Input();
		Calculate_PID(); 
		Calculate_Motors_Speed();
		Set_Motors_Speed();

		sprintf(str, "pitch: %f, roll: %f\r", angle_pitch_output, angle_roll_output);
		UART_0_Transmit(str, 42);                                                      //USB serial output
		//UART_1_Transmit(str, 42);                                                   //radio serial output
		
		vTaskDelayUntil(&myLastUnblock, 4 * configTICK_RATE_HZ/1000); //4 ms period (250 Hz)
		zmienna_1++;
	}
}

void myTask2(void *p){
	
	while(1){
		adcVal = (ADC_Value_Get())*0.3313;
		battery_voltage = battery_voltage*0.92 + 0.08*adcVal;
		battery_voltage = (int)battery_voltage;
		if(battery_voltage<1050) LED_ON(red_led);
		else if(battery_voltage<1155 && battery_voltage>1050) LED_ON(green_led+red_led);
		else LED_ON(green_led);
	}
}


int main(){
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); //80 MHz (MAX frequency)
	LED_Init();
	PWM_Init();
	UART_0_Init();
	UART_1_Init();
	I2C_Init();
	ADC_Init();
	Timer_Capture_Init();
	Calibrate_Motors();
	Setup_MPU6050_Registers();
	
	
	xTaskCreate(myTask1, "task1", 200, (void*) 0, 2, &myTask1Handle);
	xTaskCreate(myTask2, "task2", 200, (void*) 0, tskIDLE_PRIORITY, &myTask2Handle);
	vTaskStartScheduler();
	
	while(1);
}

