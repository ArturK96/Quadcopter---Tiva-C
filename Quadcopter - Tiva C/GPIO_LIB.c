#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "GPIO_LIB.h"
#include "InterruptHandlers_LIB.h"

void GPIO_Input(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_DIR_MODE_IN);
}

void GPIO_Input_Interrupt_Init(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlDelay(3);
		
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3);
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  
  //GPIOPinTypeGPIOOutput(GPIO_PORT_BASE, GPIO_PIN_1);
	
	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_FALLING_EDGE);
	GPIOIntRegister(GPIO_PORTD_BASE,PortDIntHandler);
	
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_2|GPIO_INT_PIN_3);
}
