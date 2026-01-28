/*
 * osKernel.c
 *
 *  Created on: Jan 21, 2026
 *      Author: henryhungwy
 */


#include "osKernel.h"
#include "cpu.h
#define NUM_OF_THREADS 3
#define STACK_SIZE 400
#define BUS_FREQ 16000000

#define CTRL_ENABLE (1U<<0)
#define CTRL_CLCKSRC (1U<<2)
#define CTRL_TICKINT (1U<<1)
#define CTRL_COUNTFLAG (1U<<16)
#define SYSTICK_RST  0
#define TIM2EN 		 (1U<<0)
#define CR1_CEN		 (1U<<0)
#define DIER_UIE	 (1U<<0)

#define INTCTRL (*(volatile uint32_t *)0xE000ED04)
#define PENDSTET	 (1U<<26)

uint32_t period_tick;

void osSchedulerLaunch(void);
void osSchedulerRoundRobin(void);

uint32_t MILIS_PRESCALER;


struct tcb{
	int32_t *stackPt;
	struct tcb *nextPt;
	thread_state_t state;
};

typedef enum {
  THREAD_READY = 0,
  THREAD_BLOCKED = 1
} thread_state_t;


typedef struct {
  int32_t count;
  tcbType *wait_list_head;
  tcbType *wait_list_tail;
} osSemaphore_t;




typedef struct tcb tcbType;
tcbType tcbs[NUM_OF_THREADS];
static tcbType *currentPt;
static int32_t TCB_STACK[NUM_OF_THREADS][STACK_SIZE];


void osKernelStackInit(int i){
	tcbs[i].stackPt=&TCB_STACK[i][STACK_SIZE-16]; //Set Stack Pointers
	tcbs[i].state = THREAD_READY;

	TCB_STACK[i][STACK_SIZE-1]=(1U<<24); //Set PSR to operate in Thumb Mode


	//Debugging Stack Content
	TCB_STACK[i][STACK_SIZE-3]= 0xFFFFFFFD; //R14 LR
	TCB_STACK[i][STACK_SIZE-4]= 0xAAAAAAAA; //R12
	TCB_STACK[i][STACK_SIZE-5]= 0xAAAAAAAA; //R3
	TCB_STACK[i][STACK_SIZE-6]= 0xAAAAAAAA; //R2
	TCB_STACK[i][STACK_SIZE-7]= 0xAAAAAAAA; //R1
	TCB_STACK[i][STACK_SIZE-8]= 0xAAAAAAAA; //R0

	//Not saved in stack frame
	TCB_STACK[i][STACK_SIZE-9]= 0xAAAAAAAA; //R11
	TCB_STACK[i][STACK_SIZE-10]=0xAAAAAAAA; //R10
	TCB_STACK[i][STACK_SIZE-11]=0xAAAAAAAA; //R9
	TCB_STACK[i][STACK_SIZE-12]=0xAAAAAAAA; //R8
	TCB_STACK[i][STACK_SIZE-13]=0xAAAAAAAA; //R7
	TCB_STACK[i][STACK_SIZE-14]=0xAAAAAAAA; //R6
	TCB_STACK[i][STACK_SIZE-15]=0xAAAAAAAA; //R5
	TCB_STACK[i][STACK_SIZE-16]=0xAAAAAAAA;//R4

}

uint8_t osKernelAddThreads(void(*task0)(void), void(*task1)(void), void(*task2)(void)){
	__disable_irq();

	tcbs[0].nextPt=&tcbs[1];
	tcbs[1].nextPt=&tcbs[2];
	tcbs[2].nextPt=&tcbs[0];

	//initialize stacks and Program counters
	osKernelStackInit(0);
	TCB_STACK[0][STACK_SIZE-2]=(uint32_t)(task0);

	osKernelStackInit(1);
	TCB_STACK[1][STACK_SIZE-2]=(uint32_t)(task1);

	osKernelStackInit(2);
	TCB_STACK[2][STACK_SIZE-2]=(uint32_t)(task2);

	currentPt=&tcbs[0];
	__enable_irq();
	return 1;
}

void osKernelInit(void){
	MILIS_PRESCALER=(BUS_FREQ/1000);
}

void osKernelLaunch(uint32_t quanta){
	SysTick->CTRL = SYSTICK_RST;
	SysTick->VAL=0;
	SysTick->LOAD=(quanta*MILIS_PRESCALER)-1;

	NVIC_SetPriority(SysTick_IRQn, 15);

	SysTick->CTRL = CTRL_CLCKSRC | CTRL_ENABLE;
	SysTick->CTRL |= CTRL_TICKINT;
	osSchedulerLaunch();

}

__attribute__((naked)) void SysTick_Handler(void){


	__asm("CPSID I");
	__asm("PUSH {R4-R11}"); //Save non-stack frame regs
	__asm("LDR R0, =currentPt"); //Load SP to r0

	__asm("LDR R1, [R0]"); //Set address=R0 to currentPt
	__asm("STR SP, [R1]"); //Store SP into TCB at address=R1 (tcb)

	__asm("PUSH {R0, LR}");
	__asm("BL osSchedulerRoundRobin");
	__asm("POP {R0, LR}");

	__asm("LDR R1, [R0]"); //r1=currenpt address
	__asm("LDR SP, [R1]"); //sp = to currentPt->SP
	__asm("POP {R4-R11}"); // restore R4-R11
	__asm("CPSIE	I");
	__asm("BX LR");

}


void osSchedulerLaunch(void){
	__asm("LDR R0, =currentPt"); //Load address of currentPt to R0
	__asm("LDR R2, [r0]"); // R2= Current PT
	__asm("LDR SP, [R2]"); //Load cortex-m SP from adress of R2 (SP=currentPt->SP)
	__asm("POP {R4-R11}");
	__asm("POP {R12}");
	__asm("POP {R0-R3}");
	__asm("ADD SP, SP, #4"); //SKIP LR
	__asm("POP {LR}"); //Create new start location by popping LR
	__asm("ADD SP, SP, #4"); //SKIP LR

	__asm("CPSIE	I");
	__asm("BX 	LR"); //Return from exception
}



void osSchedulerRoundRobin(void){
	if ((++period_tick)==PERIOD){
		(*task3)();
	}
	currentPt=currentPt->nextPt;
}


void tim2_1hz_interrupt_init(void){
	RCC->APB1ENR|=TIM2EN;
	TIM2->PSC= 1600-1;
	TIM2->ARR=10000;
	TIM2->CNT=0;
	TIM2->CR1= CR1_CEN;
	TIM2->DIER|=DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
}

void osSemaphoreInit(int32_t *semaphore, int32_t value){
	*semaphore=value;
}

void osSemaphoreSet(int32_t *semaphore){
	__disable_irq();
	tcbType *t = sem_dequeue(sem);
	if (t) {
	    t->state = THREAD_READY;
	  }
	else {
	    sem->count++; //Increment
	  }

	__enable_irq();
}

void osSemaphoreWait(int32_t *semaphore){
	__disable_irq();
	while(*semaphore<=0){
		__disable_irq();
		__enable_irq();
	}
	*semaphore-=1;
	__enable_irq();
}
