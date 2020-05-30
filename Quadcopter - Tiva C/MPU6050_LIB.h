#include <stdint.h>
#include <stdbool.h>

//extern float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

///////////////////////////////////////////////////////////////////////
extern int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
extern float angle_pitch_output, angle_roll_output;
extern float angle_pitch, angle_roll;
extern double gyro_pitch, gyro_roll, gyro_yaw;
///////////////////////////////////////////////////////////////////////

void MPUtestConnection(void);
void MPU6050_Init(void);
void readMPU(void);
float getMPUangleX(void);
float getMPUangleY(void);
void MPU6050_Read_Data(void);
/////////////////////////////////////////////////////
void set_gyro_registers(void);
void read_mpu_6050_data(void);
void calculate_mpu_6050_angles(void);