#include "bus.h"
#include <string.h>
#include "rom.h"
#include "uart.h"
#include "gpio.h"

uint8_t cpu_ram[1024*24];

void bus_reset(void){
    memset(cpu_ram, 0, sizeof(cpu_ram));

}

uint8_t bus_read(uint16_t addr){
    //24KB of virtual ram
    if (addr>=0x0000 && addr<=0x5FFF){
        return cpu_ram[addr];
    }
    //4KB of virtual rom
    if (addr>=0xF000 && addr<=0xFFFF){
        return rom[addr-0xF000];
    }
    else{
         return 0x00;
    }
}

void bus_write(uint16_t addr, uint8_t value){
    if (addr>=0x0000 && addr<=0x5FFF){
        cpu_ram[addr]=value;
    }
    //4KB of virtual rom

    else if (addr==0x8000){
        gpio_toggle_led();
    }
    else if (addr==0xF001){
        uart_write(value);
    }
    else if (addr>=0xF000 && addr<=0xFFFF){
        //ROM
        }
    else{
        //DO nothing
    }
}
