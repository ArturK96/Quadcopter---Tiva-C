#include "PID_LIB.h"

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
