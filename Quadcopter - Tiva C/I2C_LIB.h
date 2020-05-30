#include <stdint.h>
#include <stdbool.h>

void I2C_Init(void);
void I2C_Read(uint8_t slave_addr, uint8_t reg, int *data);
void I2C_Write(uint8_t slave_addr, uint8_t reg, uint8_t data);
