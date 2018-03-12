#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "Delay.h"
#include "stm32f10x_i2c.h"


/*******************************************************************/
#define _I2C_SR2_BUSY           1

#define PIN6                            6
#define PIN7                            7
#define CR_PIN6                         (PIN6 << 2)
#define CR_PIN7                         (PIN7 << 2)

#define CR_MODE6                        CR_PIN6
#define CR_MODE7                        CR_PIN7

#define CR_CNF6                         (CR_PIN6 + 2)
#define CR_CNF7                         (CR_PIN7 + 2)

extern void TestI2CAccess();

