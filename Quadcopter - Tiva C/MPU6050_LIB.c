#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "I2C_LIB.h"
#include "LED_LIB.h"
#include "MPU6050_LIB.h"


int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z, acc_total_vector;
int16_t mag_x, mag_y, mag_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal = 0;
long acc_x_cal, acc_y_cal, acc_z_cal = 0;
float angle_pitch, angle_roll;
bool set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
double gyro_pitch, gyro_roll, gyro_yaw;
float gyro_pitch_input, gyro_roll_input, gyro_yaw_input;

int accXout_L, accXout_H, accXout;
int accYout_L, accYout_H, accYout;
int accZout_L, accZout_H, accZout;

int gyroXout_L, gyroXout_H, gyroXout;
int gyroYout_L, gyroYout_H, gyroYout;
int gyroZout_L, gyroZout_H, gyroZout;

int magXout_L, magXout_H, magXout;
int magYout_L, magYout_H, magYout;
int magZout_L, magZout_H, magZout;


//MPU-6050 addresses
#define MPU_ADDRESS		0x68 // AD0 pin na 3.3 V in pol je 0x69, cene nevem (oz niti ne dela (?????))

#define WHO_AM_I		0x75
#define PWR_MGMT_1		0x6B
#define SMPRT_DIV		0x19
#define CONFIG			0x1A
#define GYRO_CONFIG		0x1B
#define ACC_CONFIG		0x1C
#define INT_PIN_CFG		0x37
#define INT_ENABLE		0x38

#define ACCEL_XOUT_H	0x3B
#define ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define ACCEL_ZOUT_L	0x40
#define TEMP_OUT_H    0x41
#define TEMP_OUT_L    0x42
#define GYRO_XOUT_H		0x43
#define GYRO_XOUT_L		0x44
#define GYRO_YOUT_H		0x45
#define GYRO_YOUT_L		0x46
#define GYRO_ZOUT_H		0x47
#define GYRO_ZOUT_L		0x48


//LSM9DS1 addresses
#define LSM9DS1_ADDRESS_ACCELGYRO 0x6B //0x6A
#define LSM9DS1_ADDRESS_MAG 0x1E       //0x1C

//accel and gyro addresses
#define CTRL_REG5_XL 0x1F
#define CTRL_REG6_XL 0x20
#define CTRL_REG1_G 0x10
#define CTRL_REG4 0x1E
#define CTRL_REG8 0x22

#define OUT_X_G_H 0x18
#define OUT_X_G_L 0x19
#define OUT_Y_G_H 0x1A
#define OUT_Y_G_L 0x1B
#define OUT_Z_G_H 0x1C
#define OUT_Z_G_L 0x1D

#define OUT_X_XL_H 0x28
#define OUT_X_XL_L 0x29
#define OUT_Y_XL_H 0x2A
#define OUT_Y_XL_L 0x2B
#define OUT_Z_XL_H 0x2C
#define OUT_Z_XL_L 0x2D

//magnetometer addresses
#define WHO_AM_I_M 0x0F
#define CTRL_REG1_M 0x20
#define CTRL_REG2_M 0x21
#define CTRL_REG3_M 0x22
#define CTRL_REG4_M 0x23
#define CTRL_REG5_M 0x24

#define OUT_X_M_H 0x28
#define OUT_X_M_L 0x29
#define OUT_Y_M_H 0x2A
#define OUT_Y_M_L 0x2B
#define OUT_Z_M_H 0x2C
#define OUT_Z_M_L 0x2D



void Setup_MPU6050_Registers(void){
  //Setup the MPU-6050
	I2C_Write(MPU_ADDRESS, PWR_MGMT_1, 0x00);

	I2C_Write(MPU_ADDRESS, GYRO_CONFIG, 0x08);
	I2C_Write(MPU_ADDRESS, ACC_CONFIG, 0x10);
	I2C_Write(MPU_ADDRESS, CONFIG, 0x03);
	
	for (long cal_int = 0; cal_int < 10000 ; cal_int ++){                  //Run this code 2000 times
		Read_MPU6050_Data();                                              //Read the raw acc and gyro data from the MPU-6050
		gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
		gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
		gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
		acc_x_cal += acc_x;
		acc_y_cal += acc_y;
		acc_z_cal += acc_z;
		//SysCtlDelay(200000);//delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
//  gyro_x_cal /= 10000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
//  gyro_y_cal /= 10000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
//  gyro_z_cal /= 10000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
//	acc_x_cal /= 400000;
//	acc_y_cal /= 400000;
//	acc_z_cal /= 400000;

	gyro_x_cal = -45;                                                 
  gyro_y_cal = -39;                                                 
  gyro_z_cal = 22;                                                  
	acc_x_cal = 266;
	acc_y_cal = 22;
	acc_z_cal = -218; //3998 -4433

}  

void Read_MPU6050_Data(void){
	
	I2C_Read(MPU_ADDRESS, ACCEL_XOUT_H, &accXout_H);
	I2C_Read(MPU_ADDRESS, ACCEL_XOUT_L, &accXout_L);
	I2C_Read(MPU_ADDRESS, ACCEL_YOUT_H, &accYout_H);
	I2C_Read(MPU_ADDRESS, ACCEL_YOUT_L, &accYout_L);
	I2C_Read(MPU_ADDRESS, ACCEL_ZOUT_H, &accZout_H);
	I2C_Read(MPU_ADDRESS, ACCEL_ZOUT_L, &accZout_L);

	I2C_Read(MPU_ADDRESS, GYRO_XOUT_H, &gyroXout_H);
	I2C_Read(MPU_ADDRESS, GYRO_XOUT_L, &gyroXout_L);
	I2C_Read(MPU_ADDRESS, GYRO_YOUT_H, &gyroYout_H);
	I2C_Read(MPU_ADDRESS, GYRO_YOUT_L, &gyroYout_L);
	I2C_Read(MPU_ADDRESS, GYRO_ZOUT_H, &gyroZout_H);
	I2C_Read(MPU_ADDRESS, GYRO_ZOUT_L, &gyroZout_L);

	acc_x = ((accXout_H << 8) | accXout_L);
	acc_y = ((accYout_H << 8) | accYout_L);
	acc_z = ((accZout_H << 8) | accZout_L);

	gyro_x = ((gyroXout_H << 8) | gyroXout_L);
	gyro_y = ((gyroYout_H << 8) | gyroYout_L);
	gyro_z = ((gyroZout_H << 8) | gyroZout_L);
}

void Calculate_MPU6050_Angles(void){

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
//	acc_x -= acc_x_cal;
//	acc_y -= acc_y_cal;
//	acc_z -= acc_z_cal;
	
  gyro_pitch = gyro_x;
	gyro_roll = gyro_y;
	gyro_yaw = gyro_z;
	
	gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 65.5) * 0.2);   //Gyro pid input is deg/sec.
	gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 65.5) * 0.2);//Gyro pid input is deg/sec.
	gyro_yaw_input = -((gyro_yaw_input * 0.8) + ((gyro_yaw / 65.5) * 0.2));      //Gyro pid input is deg/sec.
	
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 2.594206-4.3;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= -5.997804;;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

	//Round to first point decimal 
	angle_pitch_output = ((float)((int)(angle_pitch_output*10)))/10;
	angle_roll_output = ((float)((int)(angle_roll_output*10)))/10;
	
	
}

void Setup_LSM9DS1_Registers(void){
  //Setup the LSM9DS1
	I2C_Write(LSM9DS1_ADDRESS_ACCELGYRO, CTRL_REG4, 0x38); //enable xyz axes gyroscope
	I2C_Write(LSM9DS1_ADDRESS_ACCELGYRO, CTRL_REG1_G, 0xCB); //500 dps
	I2C_Write(LSM9DS1_ADDRESS_ACCELGYRO, CTRL_REG5_XL, 0x38);
	I2C_Write(LSM9DS1_ADDRESS_ACCELGYRO, CTRL_REG6_XL, 0xD8); // +-8g
	I2C_Write(LSM9DS1_ADDRESS_ACCELGYRO, CTRL_REG8, 0x06);
	
	I2C_Write(LSM9DS1_ADDRESS_MAG, CTRL_REG1_M, 0x7E); // select x,y-axis mode
	I2C_Write(LSM9DS1_ADDRESS_MAG, CTRL_REG2_M, 0x00); // select mag full scale +-4 gauss
	I2C_Write(LSM9DS1_ADDRESS_MAG, CTRL_REG3_M, 0x00); // continuous conversion mode
	I2C_Write(LSM9DS1_ADDRESS_MAG, CTRL_REG4_M, 0x07); // select z-axis mode
	I2C_Write(LSM9DS1_ADDRESS_MAG, CTRL_REG5_M, 0x00); // select block update mode
	
	for (long cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Run this code 2000 times
		Read_LSM9DS1_Data();                                              //Read the raw acc and gyro data from the MPU-6050
		gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
		gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
		gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
//		acc_x_cal += acc_x;
//		acc_y_cal += acc_y;
//		acc_z_cal += acc_z;
		//SysCtlDelay(200000);//delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 1000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 1000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 1000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
//	acc_x_cal /= 1000;
//	acc_y_cal /= 1000;
//	acc_z_cal /= 1000;

	gyro_x_cal = 61;                                                 
  gyro_y_cal = 76;                                                 
  gyro_z_cal = 49;                                                  
//	acc_x_cal = -76;
//	acc_y_cal = 99;
//	acc_z_cal = 150; //4326 
}

void Read_LSM9DS1_Data(void){
	
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_X_XL_H, &accXout_H);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_X_XL_L, &accXout_L);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_Y_XL_H, &accYout_H);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_Y_XL_L, &accYout_L);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_Z_XL_H, &accZout_H);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_Z_XL_L, &accZout_L);

	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_X_G_H, &gyroXout_H);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_X_G_L, &gyroXout_L);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_Y_G_H, &gyroYout_H);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_Y_G_L, &gyroYout_L);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_Z_G_H, &gyroZout_H);
	I2C_Read(LSM9DS1_ADDRESS_ACCELGYRO, OUT_Z_G_L, &gyroZout_L);
	
	I2C_Read(LSM9DS1_ADDRESS_MAG, OUT_X_M_H, &magXout_H);
	I2C_Read(LSM9DS1_ADDRESS_MAG, OUT_X_M_L, &magXout_L);
	I2C_Read(LSM9DS1_ADDRESS_MAG, OUT_Y_M_H, &magYout_H);
	I2C_Read(LSM9DS1_ADDRESS_MAG, OUT_Y_M_L, &magYout_L);
	I2C_Read(LSM9DS1_ADDRESS_MAG, OUT_Z_M_H, &magZout_H);
	I2C_Read(LSM9DS1_ADDRESS_MAG, OUT_Z_M_L, &magZout_L);

	acc_x = ((accXout_H << 8) | accXout_L);
	acc_y = ((accYout_H << 8) | accYout_L);
	acc_z = ((accZout_H << 8) | accZout_L);

	gyro_x = ((gyroXout_H << 8) | gyroXout_L);
	gyro_y = ((gyroYout_H << 8) | gyroYout_L);
	gyro_z = ((gyroZout_H << 8) | gyroZout_L);
	
	mag_x = ((magXout_H << 8) | magXout_L);
	mag_y = ((magYout_H << 8) | magYout_L);
	mag_z = ((magZout_H << 8) | magZout_L);
}

void Calculate_LSM9DS1_Angles(void){

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
	acc_x -= acc_x_cal;
	acc_y -= acc_y_cal;
	acc_z -= acc_z_cal;
	
  gyro_pitch = gyro_x;
	gyro_roll = gyro_y;
	gyro_yaw = gyro_z;
	
	gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll * 0.0175) * 0.2);   //Gyro pid input is deg/sec.
	gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch * 0.0175) * 0.2);//Gyro pid input is deg/sec.
	gyro_yaw_input = -((gyro_yaw_input * 0.8) + ((gyro_yaw * 0.0175) * 0.2));      //Gyro pid input is deg/sec.
	
  //Gyro angle calculations
  //0.00007 = (0.0175dps/LSB) / 250Hz
  angle_pitch += gyro_x * 0.00007;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.00007;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.0000048876 = 0.00028 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001222);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001222);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* -57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* 57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

	//Round to first point decimal 
	angle_pitch_output = ((float)((int)(angle_pitch_output*10)))/10;
	angle_roll_output = ((float)((int)(angle_roll_output*10)))/10;
}
