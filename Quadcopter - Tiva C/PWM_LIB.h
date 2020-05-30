#include <stdbool.h>
#include <stdint.h>

extern int period;

void PWM_Init(void);
void PWM_Set_Value_FL_Motor(float value);
void PWM_Set_Value_BL_Motor(float value);
void PWM_Set_Value_FR_Motor(float value);
void PWM_Set_Value_BR_Motor(float value);
