#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "LED_LIB.h"
#include "InterruptHandlers_LIB.h"
#include "mat.h"

uint8_t value, flag_00;
uint32_t steps, numer, czas_1, czas_2, freq_int;
float freq, freq_1, freq_2;
int32_t channel_0, channel_1, channel_2, channel_3;
uint32_t start_0, start_1, start_2, start_3, end_0, end_1, end_2, end_3;
float freq_start, freq_end_0, freq_end_1, freq_end_2, freq_end_3;

void PortDIntHandler(void){
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_2|GPIO_INT_PIN_3);
	value = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_3|GPIO_PIN_2);
	if(value == 0x00){ 
		flag_00=1;
		LED_OFF(blue_led);
	}
	if(value == 0x04 && flag_00==1){ 
		steps++, flag_00=0, LED_TOGGLE(red_led), LED_ON(blue_led);
	}
	//uart_msg[i] = steps/10UART_Transmit("dupab", 5)
}

void TimerCaptureIntHandler_0(void){
	TimerIntClear(WTIMER0_BASE, (TIMER_TIMA_TIMEOUT | TIMER_CAPA_EVENT ));
	TimerIntClear(WTIMER0_BASE, (TIMER_TIMB_TIMEOUT | TIMER_CAPB_EVENT ));
	LED_TOGGLE(blue_led);
	channel_0 = TimerValueGet(WTIMER0_BASE, TIMER_B)/80;
	HWREG(WTIMER0_BASE+TIMER_O_TBV)=0;
}

void TimerCaptureIntHandler_1(void){
	TimerIntClear(WTIMER1_BASE, (TIMER_TIMA_TIMEOUT | TIMER_CAPA_EVENT ));
	TimerIntClear(WTIMER1_BASE, (TIMER_TIMB_TIMEOUT | TIMER_CAPB_EVENT ));
	LED_TOGGLE(blue_led);
	channel_1 = mapf(TimerValueGet(WTIMER1_BASE, TIMER_B)/80, 1000, 2000, 2000, 1000);
	HWREG(WTIMER1_BASE+TIMER_O_TBV)=0;
}

void TimerCaptureIntHandler_2(void){
	TimerIntClear(WTIMER3_BASE, (TIMER_TIMA_TIMEOUT | TIMER_CAPA_EVENT ));
	TimerIntClear(WTIMER3_BASE, (TIMER_TIMB_TIMEOUT | TIMER_CAPB_EVENT ));
	LED_TOGGLE(blue_led);
	channel_2 = TimerValueGet(WTIMER3_BASE, TIMER_B)/80;
	HWREG(WTIMER3_BASE+TIMER_O_TBV)=0;
}

void TimerCaptureIntHandler_3(void){
	TimerIntClear(WTIMER5_BASE, (TIMER_TIMA_TIMEOUT | TIMER_CAPA_EVENT ));
	TimerIntClear(WTIMER5_BASE, (TIMER_TIMB_TIMEOUT | TIMER_CAPB_EVENT ));
	LED_TOGGLE(blue_led);
	channel_3 = TimerValueGet(WTIMER5_BASE, TIMER_B)/80;
	HWREG(WTIMER5_BASE+TIMER_O_TBV)=0;
}
