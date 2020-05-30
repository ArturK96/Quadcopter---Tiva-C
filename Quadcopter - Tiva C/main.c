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
#include "MadgwickAHRS.h"


uint32_t SystemCoreClock = 80000000;

TaskHandle_t myTask1Handle = NULL;
TaskHandle_t myTask2Handle = NULL;
TaskHandle_t myTask3Handle = NULL;
TaskHandle_t myTask4Handle = NULL;

QueueHandle_t myQueue;
bool start = false;
float pitch, roll;
char str[42];
float adcVal, battery_voltage;
float Power_FL, Power_FR, Power_BL, Power_BR;
int32_t pwmVal, throttle, yaw_remote;
int32_t rollPWM, pitchPWM, pitch_PID_input, roll_PID_input, pitch_remote, roll_remote;
uint32_t zmienna_1, zmienna_3;

////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)
float pid_p_gain_pitch = 1.3;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.04;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 18.0;  //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;          //Maximum output of the PID-controller (+/-)
float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
void calculate_pid();
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
float roll_level_adjust, pitch_level_adjust;
////////////////////////////////////////////////////////////////////////////////////////


void myTask1(void *p){
	TickType_t myLastUnblock;
	myLastUnblock = xTaskGetTickCount();
	
	while(1){
//		MPU6050_Read_Data();
//		
//		MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
//		roll = (asinf(-2.0f * (q1*q3 - q0*q2))*(180/3.14));
//		pitch = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*(180/3.14);
//		yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*(180/3.14);
		
		gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 65.5) * 0.2);   //Gyro pid input is deg/sec.
		gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 65.5) * 0.2);//Gyro pid input is deg/sec.
		gyro_yaw_input = -((gyro_yaw_input * 0.8) + ((gyro_yaw / 65.5) * 0.2));      //Gyro pid input is deg/sec.
		
		read_mpu_6050_data();
		calculate_mpu_6050_angles();
		
		throttle = channel_0;
//		pitch_remote = channel_1;
//		roll_remote = channel_3;
//		yaw_remote = channel_2;
		
		if((channel_2<1050) && (channel_0<1050)){
			start = false;
			pid_i_mem_roll = 0;
			pid_last_roll_d_error = 0;
			pid_i_mem_pitch = 0;
			pid_last_pitch_d_error = 0;
			pid_i_mem_yaw = 0;
			pid_last_yaw_d_error = 0;
		}
		else if((channel_2>1950) && (channel_0<1050)){
			start = true;
		}

		
		pitch_level_adjust = 0;//angle_pitch_output * 15;                                    //Calculate the pitch angle correction
		roll_level_adjust = 0;//angle_roll_output * 15;                                      //Calculate the roll angle correction
		
		receiver_input_channel_1 = channel_3;
		pid_roll_setpoint = 0;
		if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
		else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;
		pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
		pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

		
		receiver_input_channel_2 = channel_1;
		pid_pitch_setpoint = 0;
		if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
		else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;
		pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
		pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

		receiver_input_channel_3 = channel_0;
		receiver_input_channel_4 = channel_2;
		pid_yaw_setpoint = 0;
		if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
			if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
			else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
		}
		
		calculate_pid(); 
		
		if(start){                                                          //The motors are started.
			if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
			Power_FR = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
			Power_BR = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
			Power_BL = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
			Power_FL = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

			if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
				Power_FL += Power_FL * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
				Power_BL += Power_BL * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
				Power_BR += Power_BR * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
				Power_FR += Power_FR * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
			} 

			if (Power_FL < 1100) Power_FL = 1100;                                         //Keep the motors running.
			if (Power_BL < 1100) Power_BL = 1100;                                         //Keep the motors running.
			if (Power_BR < 1100) Power_BR = 1100;                                         //Keep the motors running.
			if (Power_FR < 1100) Power_FR = 1100;                                         //Keep the motors running.

			if(Power_FL > 2000)Power_FL = 2000;                                           //Limit the esc-1 pulse to 2000us.
			if(Power_BL > 2000)Power_BL = 2000;                                           //Limit the esc-2 pulse to 2000us.
			if(Power_BR > 2000)Power_BR = 2000;                                           //Limit the esc-3 pulse to 2000us.
			if(Power_FR > 2000)Power_FR = 2000;                                           //Limit the esc-4 pulse to 2000us.  
			}

		else{
			Power_FL = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
			Power_BL = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
			Power_BR = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
			Power_FR = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
		}
		
		PWM_Set_Value_FL_Motor(Power_FL);
		PWM_Set_Value_FR_Motor(Power_FR);
		PWM_Set_Value_BL_Motor(Power_BL);
		PWM_Set_Value_BR_Motor(Power_BR);	
		

		
		sprintf(str, "pitch: %f, gyro: %f\r", angle_pitch_output, gyro_pitch_input);
		//UART_0_Transmit(str, 42);
		UART_1_Transmit(str, 42);
		
		LED_TOGGLE(blue_led);
		vTaskDelayUntil(&myLastUnblock, 4 * configTICK_RATE_HZ/1000); //4 ms period (250 Hz)
		zmienna_1++;
	}
}

void myTask2(void *p){
	
	while(1){
			adcVal = (ADC_Value_Get())*0.593;
			battery_voltage = battery_voltage*0.92 + 0.08*adcVal;
			battery_voltage = (int)battery_voltage;
			if(battery_voltage<1050 && battery_voltage>600) LED_ON(red_led);
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
//	MPU6050_Init();
	battery_voltage = 1260;
	
	PWM_Set_Value_FL_Motor(2000);
	PWM_Set_Value_FR_Motor(2000);
	PWM_Set_Value_BL_Motor(2000);
	PWM_Set_Value_BR_Motor(2000);
	SysCtlDelay(100000000);
	
	PWM_Set_Value_FL_Motor(1000);
	PWM_Set_Value_FR_Motor(1000);
	PWM_Set_Value_BL_Motor(1000);
	PWM_Set_Value_BR_Motor(1000);
	SysCtlDelay(100000000);
	
/////////////////////////////////////////////////////////////////////
	set_gyro_registers();
/////////////////////////////////////////////////////////////////////
	
	xTaskCreate(myTask1, "task1", 200, (void*) 0, 2, &myTask1Handle);
	xTaskCreate(myTask2, "task2", 200, (void*) 0, tskIDLE_PRIORITY, &myTask2Handle);

	
	vTaskStartScheduler();
	
	
	while(1);
	
}


////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

