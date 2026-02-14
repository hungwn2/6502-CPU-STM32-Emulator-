#include "cpu.h"
#include "bus.h"
#include "rom.h"
#include "oskernel.h"
#include <stdint.h>
#include <string.h>

extern uint32_t g_ram[256]; //Shared ram of main
extern cpu_t g_cpu;
sem_t gpio_lock, uart_lock;

static inline void set_zn_flag(cpu_t *cpu, uint8_t value){
	if (value==0) cpu->p|=Z_FLAG;
	else cpu->p &=~Z_FLAG;
	if (value & 0x80) cpu->p|=N_FLAG;
	else cpu->p &=~N_FLAG;
}

static inline void op_adc(cpu_t *cpu, uint8_t operand){
	uint16_t sum=cpu->a+operand+(cpu->p & C_FLAG ? 1:0);
	if (sum>0xFF) {cpu->p |=C_FLAG;}
	else {cpu->p &=~ C_FLAG;}

	//check for overflow
	//Checks if operand and sum have same sign, and operand has a different one
	if (~(cpu->a ^ operand) & (cpu->a ^ sum) & 0x80){
		cpu->p|=V_FLAG;
	}
	else {
		cpu->p &=~ V_FLAG;
	}
	cpu->a=(uint8_t)sum;
	set_zn_flag(cpu, cpu->a);
}



static inline void push_stack(cpu_t *cpu, uint8_t value){
	bus_write(0x0100+cpu->sp, value);
	cpu->sp--;
}

static inline uint8_t pull_stack(cpu_t *cpu){
	cpu->sp++;
	return bus_read(cpu->sp+0x0100);
}


static inline uint16_t read16(uint16_t addr) {
    uint8_t lo = bus_read(addr);
    uint8_t hi = bus_read((uint16_t)(addr + 1));
    return (uint16_t)lo | ((uint16_t)hi << 8);
}




void cpu_reset(cpu_t *cpu){
	memset(cpu, 0, sizeof(cpu_t));
	uint16_t lo=bus_read(0xFFFC);
	uint16_t high=bus_read(0xFFFD);
	cpu->sp=0xFD;
	cpu->pc = (uint16_t)lo | ((uint16_t)high << 8);
	cpu->p|=U_FLAG|I_FLAG;
}

void cpu_nmi(cpu_t *cpu){
	push_stack(cpu, (cpu->pc>>8)&0xFF);
	push_stack(cpu, cpu->pc & 0xFF);

	push_stack(cpu, (cpu->p & ~B_FLAG)|U_FLAG);
	cpu->p |= I_FLAG;
	cpu->pc=read16(0xFFFA);
	cpu->cycles+=7;
}


uint8_t fetch_imn(cpu_t *cpu){
	uint8_t value=bus_read(cpu->pc);
	cpu->pc++;
	return value;
}

uint16_t fetch_abs(cpu_t *cpu){
	uint16_t lo=bus_read(cpu->pc);
	cpu->pc++;
	uint16_t high=bus_read(cpu->pc);
	cpu->pc++;
	uint16_t value=(high<<8)|lo;
	return value;

}

void os_sycalls(cpu_t *cpu){
	switch(cpu->a){
	case SYS_UART_LOCK: osSemaphoreWait(&uart_lock); break;
	case SYS_UART_UNLOCK: osSemaphoreSet(&uart_lock); break;
	case SYS_UART_WRITE: uart_write(cpu->x); break;
	case SYS_GPIO_TOGGLE:{
	    osSemaphoreWait(&gpio_lock); //Wait for gpio lock
		gpio_toggle_led();
		osSemaphoreSet(&gpio_lock); //Free GPIO resource
	}
	case SYS_YIELD: osThreadYield(); break;
	default:
	        break;
	}
}
void cpu_step(cpu_t *cpu)
{
	uint8_t opcode=bus_read(cpu->pc);
	cpu->pc++;

	switch(opcode){
		case 0xEA://NOP
		{
			cpu->cycles+=2;
			break;
		}
		case 0x4C://JMP
		{
			cpu->pc=fetch_abs(cpu);
			break;
		}
		case 0xA9://LDA IMN
		{
			cpu->a=fetch_imn(cpu);
			set_zn_flag(cpu, cpu->a);
			cpu->cycles+=2;
			break;
		}
		case 0xAD://LDA ABS
		{
			uint16_t addr=fetch_abs(cpu);
			uint8_t value=bus_read(addr);
			cpu->a=value;
			set_zn_flag(cpu, cpu->a);
			cpu->cycles+=4;
			break;
		}
		case 0xA2://LDX IMN
		{
			uint8_t value=fetch_imn(cpu);
			cpu->x=value;
			set_zn_flag(cpu, cpu->x);
			cpu->cycles+=2;
			break;
		}
		case 0xA0: //LDY IMN
		{
			uint8_t value=fetch_imn(cpu);
			cpu->y=value;
			set_zn_flag(cpu, cpu->y);
			cpu->cycles+=2;
			break;
		}
		case 0x8D: //STA ABS
		{
			uint16_t addr=fetch_abs(cpu);
			bus_write(addr, cpu->a);
			cpu->cycles+=4;
			break;
		}
		case 0x69: //ADC IMN
		{
			uint8_t operand=fetch_imn(cpu);
			op_adc(cpu, operand);
			cpu->cycles+=2;
			break;
		}
		case 0x48: //PHA
		{
			push_stack(cpu, cpu->a);
			cpu->cycles+=3;
			break;
		}
		case 0x68://PLA
		{
			cpu->a=pull_stack(cpu);
			set_zn_flag(cpu, cpu->a);
			cpu->cycles+=4;
			break;
		}
		case 0xE8: //INX
		{
			cpu->x++;
			set_zn_flag(cpu, cpu->x);
			cpu->cycles+=2;
			break;
		}
		case 0xD0://BNE
		{
			int8_t offset=(int8_t)fetch_imn(cpu);
			cpu->cycles+=2;
			if (!(cpu->p & Z_FLAG)){
				cpu->cycles++;
				uint16_t old_pc=cpu->pc;
				uint16_t new_pc=old_pc+offset;
				//if high byte changes, add another cycles
				if((old_pc & 0xFF00)!=(new_pc & 0xFF)){
					cpu->cycles++;
				}
				cpu->pc=new_pc;
			}
			break;
		}

		case 0x00: //BRK
			uint16_t ret_addr = (uint16_t)(cpu->pc+1);
			os_sycalls(cpu);


			cpu->p |=I_FLAG;
			cpu->pc = read16(0xFFCD);
			cpu->cycles+=7;
			break;

		case 0x40:
			cpu->p=pull_stack(cpu);
			uint8_t lo=pull_stack(cpu);
			uint8_t hi = pull_stack(cpu);
			cpu->pc= ((hi<<8)|lo);
			cpu->cycles+=6;
			break;
	    default:
	    {
	    	cpu->cycles+=2;
	    	break;
	    }


	}
}
