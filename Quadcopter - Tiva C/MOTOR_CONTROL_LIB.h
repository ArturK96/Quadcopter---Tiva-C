#include <stdint.h>
#include <stdbool.h>


extern float Power_FL, Power_FR, Power_BL, Power_BR;


void Calibrate_Motors(void);
void Secure_Startup (void);
void Calculate_Motors_Speed (void);
void Set_Motors_Speed(void);
