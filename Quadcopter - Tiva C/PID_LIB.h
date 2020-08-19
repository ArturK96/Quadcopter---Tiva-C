#include <stdbool.h>
#include <stdint.h>


extern float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
extern float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
extern float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;
extern float roll_level_adjust, pitch_level_adjust;
extern volatile int roll_remote, pitch_remote, yaw_remote;
extern volatile int throttle;


void Calculate_PID_Input(void);
void Calculate_PID(void);

float PID_Controller(float kp, float ki, float kd, float input, float set_point);
