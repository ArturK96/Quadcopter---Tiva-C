#include <stdbool.h>
#include <stdint.h>
#include "mat.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/pwm.h"
#include "PWM_LIB.h"

int period;

void PWM_Init(void){
	

	SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlDelay(2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlDelay(2);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB5_M0PWM3);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);
	//GPIOPinConfigure(GPIO_PC5_M0PWM7);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);//
	
	period = 50000;
	
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);
	
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	
	PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT | PWM_OUT_0_BIT |  PWM_OUT_2_BIT |  PWM_OUT_3_BIT /*| PWM_OUT_7_BIT*/, true);
}

void PWM_Set_Value_FL_Motor(float value){
	value = mapf(value, 1000., 2000., 2499, 4999);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, value);
}

void PWM_Set_Value_BL_Motor(float value){
	value = mapf(value, 1000., 2000., 2499, 4999);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, value);
}

void PWM_Set_Value_FR_Motor(float value){
	value = mapf(value, 1000., 2000., 2499, 4999);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, value);
}
// RIGHT
void PWM_Set_Value_BR_Motor(float value){
	value = mapf(value, 1000., 2000., 2499, 4999);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, value);
}
