#ifndef CPU_H
#define CPU_H

#include <stdint.h>

#include "uart.h"
#include "gpio.h"
#include "bus.h"

typedef enum{
    C_FLAG = (1<<0),
    Z_FLAG = (1<<1),
    I_FLAG = (1<<2),
    D_FLAG = (1<<3),
    B_FLAG = (1<<4),
    U_FLAG = (1<<5),//unused
    V_FLAG = (1<<6),
    N_FLAG = (1<<7)
}cpu_flags_t;

enum {
  SYS_UART_LOCK   = 1,
  SYS_UART_UNLOCK = 2,
  SYS_UART_WRITE  = 3,
  SYS_GPIO_TOGGLE = 4,
  SYS_YIELD       = 5
};

typedef struct{
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t sp;
    uint16_t pc;
    uint8_t p;
    uint64_t cycles;
}cpu_t;

void cpu_reset(cpu_t *cpu);

void cpu_step(cpu_t *cpu);


void cpu_nmi(cpu_t *cpu);

void cpu_irq(cpu_t *cpu);

#endif
