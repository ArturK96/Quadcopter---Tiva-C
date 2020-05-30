#include <stdbool.h>
#include <stdint.h>

extern uint8_t value, flag_00;
extern uint32_t steps, numer, czas, freq_int;
extern float freq;
extern int32_t channel_0, channel_1, channel_2, channel_3;
extern uint32_t start_0, start_1, start_2, start_3, end_0, end_1, end_2, end_3;

void PortDIntHandler(void);
void TimerCaptureIntHandler_0(void);
void TimerCaptureIntHandler_1(void);
void TimerCaptureIntHandler_2(void);
void TimerCaptureIntHandler_3(void);
