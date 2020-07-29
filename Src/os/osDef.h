/*
 * osDef.h
 *
 *  Created on: Feb 12, 2020
 *      Author: Tim Buckley
 */

#ifndef OS_OSDEF_H_
#define OS_OSDEF_H_

#include "os.h"

#define	LOG2(x)	(32 - __CLZ(x))

uint32_t OS_VectorTable[OS_VECTOR_TABLE_SIZE] __attribute__(( aligned (OS_VECTORTABLE_ALIGNMENT)));

TIM_HandleTypeDef *OS_Timer;

OS_Thread_t OS_Threads[OS_MAX_THREADS];
OS_Thread_t OS_IdleThread;

OS_Thread_t *OS_Current;
OS_Thread_t *OS_Next;

uint32_t OS_ReadyMask = 0;
uint32_t OS_DelayMask = 0;

uint32_t OS_TickCounter = 0;

uint32_t OS_IdleStack[OS_MIN_STACK_SIZE];

static void OS_Start(void);

static void OS_Idle(void);
static void OS_Done(void);

static void OS_Sched(void);
static void OS_SetThreadActive(OS_Thread_t *);
static OS_Thread_t* OS_GetHighestPriorityThread(void);
static OS_Thread_t* OS_FindAvailableThread(void);
static uint32_t OS_CalculatePriorityScore(OS_Thread_t *);
static void OS_SetupVectorTable(void);

///////////////////////////
/*System Calls*/
///////////////////////////

const char idleName[] = "IDLE";

const OS_Image_t idleImage = {
		.got = (void*)0x08002cc0,
		.gotSize = 29,
		.main = OS_Idle,
		.name = idleName,
		.priority = OS_PRIORITY_NONE,
		.ram = (void*)SRAM1_BASE,
		.ramSize = 400,
		.stackSize = 100
};

const uint32_t* const isr_vector = (void*)(FLASH_BASE);

#endif /* OS_OSDEF_H_ */
