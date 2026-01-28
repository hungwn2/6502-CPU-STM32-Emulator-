/*
 * osKernel.h
 *
 *  Created on: Apr 22, 2025
 *      Author: User
 */

#ifndef OSKERNEL_H_
#define OSKERNEL_H_
#include "stm32f4xx.h"
#define SR_UIF (1U<<0)
//interrupt flag
#define PERIOD 100
#include <stdint.h>
//__attribute__((naked)) void SysTick_Handler(void);
uint8_t osKernelAddThreads(void(*task0)(void), void(*task1)(void), void(*task2)(void));
void osKernelLaunch(uint32_t quanta);
void tim2_1hz_interrupt_init(void);
void osKernelInit(void);
void osSchedulerLaunch();

void osThreadYield(void);
void task3(void);
void osSemaphoreInit(int32_t *semaphore, int32_t value);
void osSemaphoreSet(int32_t *semaphore);
void osSemaphoreWait(int32_t *semaphore);
#endif /* OSKERNEL_H_ */
