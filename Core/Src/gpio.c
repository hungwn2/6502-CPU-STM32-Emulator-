
/*
 * gpio.c
 *
 *  Created on: Sep 17, 2025
 *      Author: henryhungwy
 */


#include "gpio.h"
#define GPIOAEN (1U<<0)
#define LED_BS5 (1<<5)
#define LED_BR5 (1U<<21)
#include <stdint.h>


void led_init(void){
	RCC->AHB1ENR|=GPIOAEN;
	GPIOA->MODER|=(1U<<10);
	GPIOA->MODER &=~ (1u<<11);
}

void gpio_toggle_led(void){
	GPIOA->BSRR^=LED_BS5;
}

