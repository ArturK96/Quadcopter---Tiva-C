#include <stdbool.h>
#include <stdint.h>

#define red_led 0x02
#define blue_led 0x04
#define green_led 0x08
#define led_off 0x00

void LED_Init(void);
void LED_ON(uint32_t LEDS);
void LED_OFF(uint32_t LEDS);
void LED_TOGGLE(uint32_t LEDS);
