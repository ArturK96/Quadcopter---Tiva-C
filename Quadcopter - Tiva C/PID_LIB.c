#include "PID_LIB.h"
#include "MPU6050_LIB.h"


float pid_p_gain_roll = 0.7;    //0.7;                  //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.005;  //0.005;               //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.75;  //18.75;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                              //Maximum output of the PID-controller (+/-)
float pid_p_gain_pitch = 0.7;   //0.7;              //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.005; //0.005;           //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 18.75; //18.75;          //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;                         //Maximum output of the PID-controller (+/-)
float pid_p_gain_yaw = 3;                       //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;    //0.02;        //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;       //0.0;        //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                       //Maximum output of the PID-controller (+/-)
void Calculate_PID(void);
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;
volatile int roll_remote, pitch_remote, throttle, yaw_remote;
float roll_level_adjust, pitch_level_adjust;


float PID_Controller(float kp, float ki, float kd, float input, float set_point){
	
	float error = input - set_point;

	// calculate the proportional component (current error * p scalar)
	float p_scalar = kp;
	if(p_scalar < 0) p_scalar = 0;
	float proportional = error * p_scalar;

	// calculate the integral component (summation of past errors * i scalar)
	float i_scalar = ki;
	if(i_scalar < 0) i_scalar = 0;
	static float integral = 0;
	integral += error * i_scalar;
	if(integral >  1000) integral = 1000; // limit wind-up
	if(integral < -1000) integral = -1000;

	// calculate the derivative component (change since previous error * d scalar)
	static float previous_error = 0;
	float d_scalar = kd;
	if(d_scalar < 0) d_scalar = 0;
	float derivative = (error - previous_error) * d_scalar;
	previous_error = error;
	
	float Motor_Power = proportional + integral + derivative;
	return Motor_Power;
}

void Calculate_PID_Input(void){
	pitch_level_adjust = angle_pitch_output * 15; //*15                                   //Calculate the pitch angle correction
	roll_level_adjust = angle_roll_output * 15;  //* 15                                    //Calculate the roll angle correction
	
	pid_roll_setpoint = 0;
	if(roll_remote > 1508)pid_roll_setpoint = roll_remote - 1508;
	else if(roll_remote < 1492)pid_roll_setpoint = roll_remote - 1492;
	pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
	pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

	pid_pitch_setpoint = 0;
	if(pitch_remote > 1508)pid_pitch_setpoint = pitch_remote - 1508;
	else if(pitch_remote < 1492)pid_pitch_setpoint = pitch_remote - 1492;
	pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
	pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

	pid_yaw_setpoint = 0;
	if(throttle > 1050){ //Do not yaw when turning off the motors.
		if(yaw_remote > 1508)pid_yaw_setpoint = (yaw_remote - 1508)/3.0;
		else if(yaw_remote < 1492)pid_yaw_setpoint = (yaw_remote - 1492)/3.0;
	}
}

void Calculate_PID(void){
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
