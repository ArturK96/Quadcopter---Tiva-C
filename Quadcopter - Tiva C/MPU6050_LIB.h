#include <stdint.h>
#include <stdbool.h>


extern int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z;
extern float angle_pitch_output, angle_roll_output;
extern double gyro_pitch, gyro_roll, gyro_yaw;
extern float gyro_pitch_input, gyro_roll_input, gyro_yaw_input;

void Setup_MPU6050_Registers(void);
void Read_MPU6050_Data(void);
void Calculate_MPU6050_Angles(void);

void Setup_LSM9DS1_Registers(void);
void Read_LSM9DS1_Data(void);
void Calculate_LSM9DS1_Angles(void);
