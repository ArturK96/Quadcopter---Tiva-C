#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "I2C_LIB.h"
#include "LED_LIB.h"
#include "MPU6050_LIB.h"


static int16_t gyro_x_offset = 0;
static int16_t gyro_y_offset = 0;
static int16_t gyro_z_offset = 0;
static uint32_t samples = 0;
float accel_x, accel_y, accel_z;
//float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
int16_t accel_x_raw, accel_y_raw, accel_z_raw, mpu_temp_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal = 0;
long acc_x_cal, acc_y_cal, acc_z_cal = 0;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
bool set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
double gyro_pitch, gyro_roll, gyro_yaw;
////////////////////////////////////////////////////////////////////////////////////////////////////

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

#define tip 0.001
#define tau 1.6
#define ett 0.999375195
int data_ready;
float Itemp;

int WhoAmI, RegReset;

int accXout_L, accXout_H, accXout;
int accYout_L, accYout_H, accYout;
int accZout_L, accZout_H, accZout;
int tempout_H, tempout_L;
float accXos[3] = {0,0,0};
float accYos[3]= {0,0,0};

int gyroXout_L, gyroXout_H, gyroXout;
int gyroYout_L, gyroYout_H, gyroYout;
int gyroZout_L, gyroZout_H, gyroZout;
float gyroXos[3] = {0,0,0};
float gyroYos[3] = {0,0,0};
float gyroZos;
float fiXos[3]={0,0,0};
float fiYos[3]= {0,0,0};

float kotXos;

void MPUtestConnection(void)
{
	I2C_Read(MPU_ADDRESS, WHO_AM_I, &WhoAmI);
	if(WhoAmI == 0x68)
	{
//		UARTprintf("Connection succesful ! \n");
		LED_ON(green_led);
	}
	//else RedLed(true);
}

void MPU6050_Init(void){

	I2C_Write(MPU_ADDRESS, PWR_MGMT_1, (1 << 3) | 0x03 );		// power managment setup, temp sensor OFF, sleep mode OFF ...
	I2C_Write(MPU_ADDRESS, SMPRT_DIV, 0x01);						// sample rate 1kHz
	I2C_Write(MPU_ADDRESS, CONFIG, 0x03);						// disable FSYNC, 41 Hz gyro filtering, 1 kHz sampling		??????????
	I2C_Write(MPU_ADDRESS, GYRO_CONFIG, (2 << 3));				// gyro full scale range --> 1000 deg/s (2 << 3)
	I2C_Write(MPU_ADDRESS, ACC_CONFIG, (1 << 3));				// acc full scale range  --> 4g (1 << 3)

	I2C_Write(MPU_ADDRESS, INT_PIN_CFG, 0x30); 	// Configure INT pin or 0011 0000 ??? 0x30
	I2C_Write(MPU_ADDRESS, INT_ENABLE, 0x01);	// Enable interrupt DATA READY bit


	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlDelay(3);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2); // Set as input
	
	for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
		MPU6050_Read_Data();                                              //Read the raw acc and gyro data from the MPU-6050
		gyro_x_cal += gyro_x_raw;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
		gyro_y_cal += gyro_y_raw;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
		gyro_z_cal += gyro_z_raw;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
		acc_x_cal += accel_x_raw;
		acc_y_cal += accel_y_raw;
		//SysCtlDelay(20000000);//delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
	acc_x_cal /= 2000;
	acc_y_cal /= 2000;
	acc_z_cal = -245;
}

void readMPU(void)
{
//	uint32_t status=0;
//	status = GPIOIntStatus(GPIO_PORTE_BASE,true);
//	GPIOIntClear(GPIO_PORTE_BASE, status);


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

	accXout = ((accXout_H << 8) | accXout_L);
	accYout = ((accYout_H << 8) | accYout_L);
	accZout = ((accZout_H << 8) | accZout_L);

	gyroXout = ((gyroXout_H << 8) | gyroXout_L);
	gyroYout = ((gyroYout_H << 8) | gyroYout_L);
	gyroZout = ((gyroZout_H << 8) | gyroZout_L);

	if(accXout&0x8000) accXout|=0xFFFF0000;
	if(accYout&0x8000) accYout|=0xFFFF0000;
	if(accZout&0x8000) accZout|=0xFFFF0000;

	if(gyroXout&0x8000) gyroXout|=0xFFFF0000;
	if(gyroYout&0x8000) gyroYout|=0xFFFF0000;
	if(gyroZout&0x8000) gyroZout|=0xFFFF0000;

	accXos[1]=accXos[0];
	accYos[1]=accYos[0];
	gyroXos[1]=gyroXos[0];
	gyroYos[1]=gyroYos[0];
	fiXos[2]=fiXos[1];
	fiXos[1]=fiXos[0];
	fiYos[2]=fiYos[1];
	fiYos[1]=fiYos[0];

	float a = 2*ett,
		  b = -ett*ett,
		  c = tip*ett/tau-ett+1,
		  d = ett*ett-tip*ett/tau-ett,
		  e = tip*ett;

	accXos[0] = -atan2(accXout, accZout);
	accYos[0] = -atan2(accYout, accZout);

	gyroXos[0] = (float)gyroYout * 0.00106422515365507901031932363932f;		// pi/(180*16.4)
	gyroYos[0] = -(float)gyroXout * 0.00106422515365507901031932363932f;
	gyroZos = (float)gyroZout * 0.06097478f; //degree / second

	fiXos[0] = a*fiXos[1] + b*fiXos[2] + c*accXos[0] + d*accXos[1] + e*(gyroXos[0]-gyroXos[1]);
	fiYos[0] = a*fiYos[1] + b*fiYos[2] + c*accYos[0] + d*accYos[1] + e*(gyroYos[0]-gyroYos[1]);
//	UARTprintf("%d\n", (int)(gyroZos*57));


	//***********************************************************************

}

float getMPUangleX(void){
	//UARTprintf("kot: %d", (int)kotXos*57);
	return fiXos[0]; //
}

float getMPUangleY(void){
	return -fiYos[0];
}





void MPU6050_Read_Data(void){                                             //Subroutine for reading the raw gyro and accelerometer data
	
	// extract the raw values
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

	accel_x_raw = ((accXout_H << 8) | accXout_L);
	accel_y_raw = ((accYout_H << 8) | accYout_L);
	accel_z_raw = ((accZout_H << 8) | accZout_L);

	gyro_x_raw = ((gyroXout_H << 8) | gyroXout_L);
	gyro_y_raw = ((gyroYout_H << 8) | gyroYout_L);
	gyro_z_raw = ((gyroZout_H << 8) | gyroZout_L);

	
	// calculate the offsets at power up
	if(samples < 64) {
		samples++;
		return;
	} else if(samples < 128) {
		gyro_x_offset += gyro_x_raw;
		gyro_y_offset += gyro_y_raw;
		gyro_z_offset += gyro_z_raw;
		samples++;
		return;
	} else if(samples == 128) {
		gyro_x_offset /= 64;
		gyro_y_offset /= 64;
		gyro_z_offset /= 64;
		samples++;
	} else {
		gyro_x_raw -= gyro_x_offset;
		gyro_y_raw -= gyro_y_offset;
		gyro_z_raw -= gyro_z_offset;
	}

//	gyro_x_raw -= gyro_x_cal;
//	gyro_y_raw -= gyro_y_cal;
//	gyro_z_raw -= gyro_z_cal;
	
	// convert accelerometer readings into G's
	accel_x = (accel_x_raw - acc_x_cal) / 8192.0f;
	accel_y = (accel_y_raw - acc_y_cal) / 8192.0f;
	accel_z = (accel_z_raw - acc_z_cal) / 8192.0f;

	// convert temperature reading into degrees Celsius
	float mpu_temp = mpu_temp_raw / 340.0f + 36.53f;

	// convert gyro readings into Radians per second
	gyro_x = gyro_x_raw / 939.650784f;
	gyro_y = gyro_y_raw / 939.650784f;
	gyro_z = gyro_z_raw / 939.650784f;

}

////////////////////////////////////////////////////////////////////////////////////////////
void set_gyro_registers(void){
  //Setup the MPU-6050
	I2C_Write(MPU_ADDRESS, PWR_MGMT_1, 0x00);

	I2C_Write(MPU_ADDRESS, GYRO_CONFIG, 0x08);
	I2C_Write(MPU_ADDRESS, ACC_CONFIG, 0x10);
	I2C_Write(MPU_ADDRESS, CONFIG, 0x03);
	
	for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
		read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
		gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
		gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
		gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
		acc_x_cal += acc_x;
		acc_y_cal += acc_y;
		//SysCtlDelay(20000000);//delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
	acc_x_cal /= 2000;
	acc_y_cal /= 2000;
	acc_z_cal = -245;

}  

void read_mpu_6050_data(void){
	
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

void calculate_mpu_6050_angles(void){

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
	acc_x -= acc_x_cal;
	acc_y -= acc_y_cal;
	acc_z -= acc_z_cal;
	
  gyro_pitch = gyro_x;
	gyro_roll = gyro_y;
	gyro_yaw = gyro_z;
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

}
