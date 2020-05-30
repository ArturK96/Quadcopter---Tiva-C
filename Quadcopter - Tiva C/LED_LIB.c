#include <stdbool.h>
#include <stdint.h>
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "LED_LIB.h"

uint8_t ledON;

void LED_Init(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}

void LED_ON(uint32_t LEDS){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, LEDS);
}

void LED_OFF(uint32_t LEDS){
	GPIOPinWrite(GPIO_PORTF_BASE, LEDS, 0);
}

void LED_TOGGLE(uint32_t LEDS){
	if(ledON==1) LED_OFF(LEDS), ledON=0;
	else LED_ON(LEDS), ledON=1;
	
}
