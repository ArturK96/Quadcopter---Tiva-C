#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "Timer_LIB.h"
#include "InterruptHandlers_LIB.h"

#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_CR_R         (*((volatile unsigned long *)0x40007524))
#define GPIO_PORTD_LOCK_R       (*((volatile unsigned long *)0x40007520))

void Timer_Capture_Init(void){
	
	//Timer 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinConfigure(GPIO_PC4_WT0CCP0);
	GPIOPinConfigure(GPIO_PC5_WT0CCP1);
	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	TimerDisable(WTIMER0_BASE, TIMER_A|TIMER_B);
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME_UP|TIMER_CFG_B_CAP_TIME_UP);
	
	TimerControlEvent(WTIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
	TimerControlEvent(WTIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
	
	TimerLoadSet(WTIMER0_BASE, TIMER_A|TIMER_B, (800000000));
	
	TimerIntRegister(WTIMER0_BASE, TIMER_A|TIMER_B, TimerCaptureIntHandler_0);
	TimerIntEnable(WTIMER0_BASE, (TIMER_TIMA_TIMEOUT | TIMER_CAPA_EVENT ));
	TimerIntEnable(WTIMER0_BASE, (TIMER_TIMB_TIMEOUT | TIMER_CAPB_EVENT ));
	TimerEnable(WTIMER0_BASE, TIMER_A|TIMER_B);
	
	
	//Timer 1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinConfigure(GPIO_PC6_WT1CCP0);
	GPIOPinConfigure(GPIO_PC7_WT1CCP1);
	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6|GPIO_PIN_7);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
	TimerDisable(WTIMER1_BASE, TIMER_A|TIMER_B);
	TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME_UP|TIMER_CFG_B_CAP_TIME_UP);
	
	TimerControlEvent(WTIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
	TimerControlEvent(WTIMER1_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
	
	TimerLoadSet(WTIMER1_BASE, TIMER_A|TIMER_B, (800000000));
	
	TimerIntRegister(WTIMER1_BASE, TIMER_A|TIMER_B, TimerCaptureIntHandler_1);
	TimerIntEnable(WTIMER1_BASE, (TIMER_TIMA_TIMEOUT | TIMER_CAPA_EVENT ));
	TimerIntEnable(WTIMER1_BASE, (TIMER_TIMB_TIMEOUT | TIMER_CAPB_EVENT ));
	TimerEnable(WTIMER1_BASE, TIMER_A|TIMER_B);
	
	
	//Timer 2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinConfigure(GPIO_PD2_WT3CCP0);
	GPIOPinConfigure(GPIO_PD3_WT3CCP1);
	GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
	TimerDisable(WTIMER3_BASE, TIMER_A|TIMER_B);
	TimerConfigure(WTIMER3_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME_UP|TIMER_CFG_B_CAP_TIME_UP);
	
	TimerControlEvent(WTIMER3_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
	TimerControlEvent(WTIMER3_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
	
	TimerLoadSet(WTIMER3_BASE, TIMER_A|TIMER_B, (800000000));
	
	TimerIntRegister(WTIMER3_BASE, TIMER_A|TIMER_B, TimerCaptureIntHandler_2);
	TimerIntEnable(WTIMER3_BASE, (TIMER_TIMA_TIMEOUT | TIMER_CAPA_EVENT ));
	TimerIntEnable(WTIMER3_BASE, (TIMER_TIMB_TIMEOUT | TIMER_CAPB_EVENT ));
	TimerEnable(WTIMER3_BASE, TIMER_A|TIMER_B);
	
	
	//Timer 3
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIO_PORTD_LOCK_R |= 0x4C4F434B;
	GPIO_PORTD_CR_R |= 0x80;
	GPIOPinConfigure(GPIO_PD6_WT5CCP0);
	GPIOPinConfigure(GPIO_PD7_WT5CCP1);
	GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
	TimerDisable(WTIMER5_BASE, TIMER_A|TIMER_B);
	TimerConfigure(WTIMER5_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME_UP|TIMER_CFG_B_CAP_TIME_UP);
	
	TimerControlEvent(WTIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
	TimerControlEvent(WTIMER5_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
	
	TimerLoadSet(WTIMER5_BASE, TIMER_A|TIMER_B, (800000000));
	
	TimerIntRegister(WTIMER5_BASE, TIMER_A|TIMER_B, TimerCaptureIntHandler_3);
	TimerIntEnable(WTIMER5_BASE, (TIMER_TIMA_TIMEOUT | TIMER_CAPA_EVENT ));
	TimerIntEnable(WTIMER5_BASE, (TIMER_TIMB_TIMEOUT | TIMER_CAPB_EVENT ));
	TimerEnable(WTIMER5_BASE, TIMER_A|TIMER_B);
}
