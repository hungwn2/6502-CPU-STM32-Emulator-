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

typedef{
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t sp;
    uint16_t pc;
    uint8_t p;
    uint64_t cycles;
};
void cpu_reset(cput_t *cpu);

void cpu_step(cpu_t *cpu,)


void cpu_nmi(cpu_t *cpu);

void cpu_irq(cpu_t *cpu);
