#include "cpu.h"
#include "bus.h"
#include <string.h>
//Set z or n flags based on value
static inline void set_zn_flag(cpu_t *cpu, uint8_t value){
    if (value==0) cpu->p|=Z_FLAG;
    else cpu->p &=~Z_FLAG;
    if (value & 0x80) cpu->p|=N_FLAG;
    else cpu->p &=~N_FLAG;
}

//STACK Functions
static inline void push_stack(cpu_t *cpu, uint8_t value){
    bus_write(0x0100+cpu->sp, value);
    cpu->sp--;
}
static inline uint8_t pull_stack(cpu_t *cpu, uint8_t value){
    cpu->sp++;
    return bus_read(cpu->sp+0x0100);
}

//ALU Logic Helpers



void cpu_reset(cpu_t *cpu){
    memset(cpu, 0, sizeof(cpu_t));
    uint16_t lo=bus_read(0xFFFC);
    uint16_t hi=bus_read(0xFFFD);
    cpu->pc=(hi<<8)|lo;
    cpu->sp=0xFD;

    //Disable interrupts
    cpu->p=U_FLAG|I_FLAG;

}


static inline void op_adc(cpu_t *cpu, uint8_t operand){
    uint16_t sum=cpu->a + operand + (cpu->p & C_FLAG ? 1:0);
    //set carry flag is sum>255
    if (sum>0xFF){
        cpu->p |=C_FLAG;
    }
    else{
        cpu-> &=~C_FLAG;
    }
    if (~(cpu->a ^ operand) & (cpu-a ^ sum) & 0x80){
        cpu->p|=V_FLAG;
    }
    else{
        cpu->p&=~V_FLAG;
    }
    //store sy

    cpu->a=(uint8_t)sum;
    set_zn_flag(cpu, cpu->a);
}

void cpu_step(cpu_t *cpu){
    uint8_t opcode=bus_read(cpu->pc);
    cpu->pc++;
    switch(opcode){

        //Use actual OPCODES
        //NOP
        case 0xEA:
            //Do nothing
            cpu->cycles+=2;
            break;
        //JMP
        case 0x4c:
            cpu->pc=fetch_abs(cpu);
            break;
        case 0xA9://LDA Immediate
            cpu->a=fetch_imn(cpu);
            set_zn_flag(cpu, cpu->a);
            cpu->cycles+=2;
            break;

        case 0xAD://LDA Absolute
            uint16_t addr=fetch_abs(cpu);
            uint8_t value=bus_read(addr);
            cpu->a=value;
            set_zn_flag(cpu, cpu->a);
            cpu->cycles+=4;
            break;
        case 0xA2: //LDX Immediate
            uint8_t value=fetch_imn(cpu);
            cpu->x=value;
            zet_zn_flags(cpu cpu->y);
            cpu->cycles+=2;
            break;
        case 0xA0: //LDY Immediate
            uint8_t value=fetch_imn(cpu);
            cpu->y=value;
            set_zn_flag(cpu, cpu->y);
            cpu->cycles+=2;
            break;
        case 0x8D: //STA Absolute
            uint16_t addr=fetch_abs(cpu);
            bus_write(addr, cpu->a);
            cpu->cycles+=4;
            break;
        case 0x69: //ADC Immediate
            uint8_t operand=fetch_imn(cpu);
            op_adc(cpu, operand);
            cpu->cycles+=2;
            break;

        //Stack Functions
        case 0x48: //PHA
            push_stack(cpu, cpu->a);
            cpu->cycles+=3;
            break;
        case 0x68: //PLA
        cpu->a=pull_stack(cpu);
        set_zn_flags(cpu, cpu->a);
        cpu->cycles+=4;
        break;
        case 0xE8: //INX
            cpu->x++;
            set_zn_flag(cpu, cpu->x);
            cpu->cycles+=2;
            break;
        case 0xD0: //BNE
        {
            int8_t offset=(int8_t)fetch_imn(cpu);
            cpu->cycles+=2;
            if (!(cpu->p & Z_FLAG)){
                //branch taken- add a cycle
                cpu->cycles++;
                uint16_t old_pc=cpu->pc;
                uint16_t new_pc=old_pc + offset;
                //If high byte of pc changes, add another cycle
                if ((old_pc & 0xFF00) != (new_pc & 0xFF00))
                {
                    cpu->cycles++;
                }
                cpu->pc=new_pc;
            }
            break;
        }

    default:
    cpu->cycles+=2;
    break;
    }
}
