/*
 * osKernel.h
 *
 *  Created on: Apr 22, 2025
 *      Author: User
 */

#ifndef OSKERNEL_H_
#define OSKERNEL_H_
#include "stm32f4xx.h"
#include "cpu.h"
#define SR_UIF (1U<<0)
//interrupt flag
#define PERIOD 100
#define NUM_OF_THREADS 3
#define STACK_SIZE 400
#define BUS_FREQ 16000000
#include <stdint.h>
//__attribute__((naked)) void SysTick_Handler(void);

typedef struct tcb tcb_t;

typedef enum {
  THREAD_READY = 0,
  THREAD_BLOCKED = 1
} thread_state_t;

typedef struct tcb{
	int32_t *stackPt;
	cpu_t* cpu;
	int32_t ram[256];
	struct tcb *nextPt;
	struct tcb *sem_next;
	thread_state_t state;
};


typedef struct{
	int count;
	tcb_t *wait_head;
}sem_t;


uint8_t osKernelAddThreads(void(*task0)(void), void(*task1)(void), void(*task2)(void));
void osKernelLaunch(uint32_t quanta);
void tim2_1hz_interrupt_init(void);
void osKernelInit(void);
void osSchedulerLaunch();

void osThreadYield(void);
void task3(void);
void osSemaphoreInit(sem_t *semaphore, int32_t value);
void osSemaphoreSet(sem_t *semaphore);
void osSemaphoreWait(sem_t *semaphore);
#endif /* OSKERNEL_H_ */
