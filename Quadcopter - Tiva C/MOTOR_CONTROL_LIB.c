#include "MOTOR_CONTROL_LIB.h"
#include "PID_LIB.h"
#include "ADC_LIB.h"
#include "PWM_LIB.h"
#include "InterruptHandlers_LIB.h"
#include "driverlib/sysctl.h"

float Power_FL, Power_FR, Power_BL, Power_BR;
bool start = false;

void Calibrate_Motors(void){
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
}

void Secure_Startup (void){
	throttle = channel_0;
	yaw_remote = channel_2;
	roll_remote = channel_3;
	pitch_remote = channel_1;
	
	if((yaw_remote<1050) && (throttle<1050)){
		start = false;
		pid_i_mem_roll = 0;
		pid_last_roll_d_error = 0;
		pid_i_mem_pitch = 0;
		pid_last_pitch_d_error = 0;
		pid_i_mem_yaw = 0;
		pid_last_yaw_d_error = 0;
	}
	else if((yaw_remote>1950) && (throttle<1050)){
		start = true;
	}
}

void Calculate_Motors_Speed (void){
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
}

void Set_Motors_Speed(void){
	PWM_Set_Value_FL_Motor(Power_FL);
	PWM_Set_Value_FR_Motor(Power_FR);
	PWM_Set_Value_BL_Motor(Power_BL);
	PWM_Set_Value_BR_Motor(Power_BR);	
}
