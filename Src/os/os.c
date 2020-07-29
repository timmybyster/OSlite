/*
 * os.c
 *
 *  Created on: Feb 12, 2020
 *      Author: Tim Buckley
 */
#include "osDef.h"

void TIM2_IRQ_Handler2(void)
{
	HAL_TIM_IRQHandler(OS_Timer);
}

void OS_Init(TIM_HandleTypeDef* thisTimer)
{
	__disable_irq();
	OS_Timer = thisTimer;
	OS_ThreadCreate("IDLE", OS_Idle, OS_PRIORITY_NONE, OS_IdleStack, OS_MIN_STACK_SIZE);
	OS_SetupVectorTable();
}

void OS_Run(void)
{
	/*Change the vector offset table to Ram so it can be set dynamically*/
	SCB->VTOR = (uint32_t)&OS_VectorTable;
	  __DSB();

	/*Get The highest priority Thread*/
	OS_Next = OS_GetHighestPriorityThread();
	OS_Current = OS_Next;

	/*Start the Timer OS timer*/
	HAL_TIM_Base_Start_IT(OS_Timer);

	/*Start the OS*/
	OS_Start();
}

static void OS_Idle(void)
{
	while(1)
	{
 		__WFI();
	}
}

static void OS_Done(void)
{
	OS_Kill();
}

static void OS_SetupVectorTable(void)
{
	for(uint16_t i = 0; i < OS_VECTOR_TABLE_SIZE; i++)
	{
		OS_VectorTable[i] = isr_vector[i];
	}

	OS_VectorTable[16 + TIM2_IRQn] = (uint32_t)&TIM2_IRQ_Handler2;
}

static void OS_Sched(void)
{
	OS_Thread_t* thisOsThread;
	uint32_t workingMask;

	/*If no thread is ready then run the idle Thread*/
	if(OS_ReadyMask == 0U)
	{
		OS_Next = &OS_IdleThread;
	}

	else
	{
		/*Assign the Next task to highest Priority*/
		OS_Next = OS_GetHighestPriorityThread();

		/*Increment any thread-that-should-be-running's readySkip counter*/
		workingMask = OS_ReadyMask;
		while(workingMask != 0U)
		{
			thisOsThread = &OS_Threads[LOG2(workingMask) - 1];
			if(thisOsThread == OS_Next)
			{
				/*For the thread about to be executed set its readySkip counter to 0*/
				thisOsThread->readySkips = 0;
			}
			else
			{
				/*Increment any other threads readySkip counter*/
				thisOsThread->readySkips++;
			}
			workingMask &= ~(1U << thisOsThread->id);
		}
	}

	/*If the current thread is not the highest priority the trigger the PENDSV Handler to switch to it*/
	if(OS_Next != OS_Current)
	{
		*(uint32_t volatile*)0xE000ED04 = (1U << 28);
	}
}

static OS_Thread_t* OS_GetHighestPriorityThread(void)
{
	OS_Thread_t* thisOsThread, *priorityThread = &OS_IdleThread;
	uint32_t workingMask = OS_ReadyMask, priorityScore = 0;

	/*Determine which is the highest priority thread*/
	while(workingMask != 0U)
	{
		thisOsThread = &OS_Threads[LOG2(workingMask) - 1];
		if(OS_CalculatePriorityScore(thisOsThread) > priorityScore)
		{
			priorityThread = thisOsThread;
			priorityScore = OS_CalculatePriorityScore(thisOsThread);
		}
		workingMask &= ~(1U << thisOsThread->id);
	}

	return priorityThread;
}

static OS_Thread_t* OS_FindAvailableThread(void)
{
	uint8_t i = 0;

	while((i != OS_MAX_THREADS) && (OS_Threads[i].active))
	{
		i++;
	}

	if(i >= OS_MAX_THREADS)
	{
		return NULL;
	}

	OS_Threads[i].id = i;

	return &OS_Threads[i];
}

static uint32_t OS_CalculatePriorityScore(OS_Thread_t* thisOsThread)
{
	return (thisOsThread->readySkips + 1)*(uint32_t)thisOsThread->priority;
}

static void OS_Tick(void)
{
	OS_Thread_t* thisOsThread;
	uint32_t bit, workingMask = OS_DelayMask;

	while(workingMask != 0U)
	{
		thisOsThread = &OS_Threads[LOG2(workingMask) - 1];
		bit = 1U << thisOsThread->id;
		thisOsThread->delay--;
		if(thisOsThread->delay == 0U)
		{
			OS_DelayMask &= ~bit;
			thisOsThread->semaphores = NULL;
			OS_SetThreadActive(thisOsThread);
		}
		workingMask &= ~bit;
	}

	OS_TickCounter++;
}

static void OS_SetThreadActive(OS_Thread_t *waitingThread)
{
	OS_Thread_t * eventThread;

	OS_ReadyMask |= 1U << waitingThread->id;

	/*Clear all the other waitingThreadMask of waitingThread id.*/
	while(waitingThread->threadMask != 0)
	{
		eventThread = &OS_Threads[LOG2(waitingThread->threadMask) - 1];
		eventThread->waitingThreadMask &= ~(1U << waitingThread->id);

		waitingThread->threadMask &= ~(1U << eventThread->id);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*If the timer is the OS Timer*/
	if(htim == OS_Timer)
	{
		/*Call the OS tick Handler*/
		OS_Tick();
		/*Check to see if a new task is ready.*/
		OS_Sched();
	}
}

extern int errno;

caddr_t _sbrk(int incr)
{
	extern char end asm("_heap");
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;
	if (heap_end + incr > (char*)0x20018000)
	{
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

__attribute__ ((naked)) static void OS_Start(void)
{
	__asm volatile("ldr r0, =0xE000ED08");	/* Use the NVIC offset register to locate the stack. */
	__asm volatile("ldr r0, [r0]");
	__asm volatile("ldr r0, [r0]");
	__asm volatile("msr msp, r0"); 			/* Set the msp back to the start of the stack. */
	__asm volatile("mov r0, #0"); 			/* Clear the bit that indicates the FPU is in use, see comment above. */
	__asm volatile("msr control, r0");
	__asm volatile("cpsie 		i");		/* Globally enable interrupts. */
	__asm volatile("svc 		0");		/* System call to start first task. */
	__asm volatile("nop");
}

__attribute__ ((naked)) void SVC_Handler(void)
{
	__asm volatile("ldr		r3, =OS_Next");		/*Get OS_Next->sp and store in R3*/
	__asm volatile("ldr		r3, [r3, #0]");
	__asm volatile("ldr		r0, [r3, #0]");		/*Set R0 to OS_Next->sp(the new process Stack Pointer)*/

	__asm volatile("ldmia 	r0!, {r4-r11, lr}");/*Pop R4-R11 from the new process stack and update R0*/
	__asm volatile("vldmia 	r0!, {s16-s31}");	/*Pop S16-S11 from the new process stack and update R0*/

	__asm volatile("msr 	psp, r0");			/*Load the value of R0(the new process stack Pointer) into PSP*/

	__asm volatile("mov 	r0, #0");			/*Clear the BASEPRI register*/
	__asm volatile("msr 	basepri, r0");

	__asm volatile("bx 		lr");
}

__attribute__ ((naked)) void PendSV_Handler(void)
{
	__asm volatile("mrs 	r0, psp");			/*Get the process stack Pointer store in R0*/
	__asm volatile("isb");						/*Clear Pipe*/

	__asm volatile("vstmdb 	r0!, {s16-s31}");	/*Push S16-S31 onto the stack with update of R0 to last stack address*/
	__asm volatile("stmdb 	r0!, {r4-r11, lr}");/*Push R4-R11 onto the stack with update of R0 to last stack address*/

	__asm volatile("ldr		r3, =OS_Current");	/*Update the currentProcess stack Pointer with the address of R0*/
	__asm volatile("ldr		r3, [r3, #0]");
	__asm volatile("str		r0, [r3, #0]");

	__asm volatile("ldr		r3, =OS_Next");		/*Set OS_Current to OS_Next*/
	__asm volatile("ldr		r3, [r3, #0]");
	__asm volatile("ldr		r2, =OS_Current");
	__asm volatile("str		r3, [r2, #0]");

	__asm volatile("ldr		r0, [r3, #0]");		/*Set R0 to OS_Next->sp(the new process Stack Pointer)*/

	__asm volatile("ldmia 	r0!, {r4-r11, lr}");/*Pop R4-R11 from the new process stack and update R0*/
	__asm volatile("vldmia 	r0!, {s16-s31}");	/*Pop S16-S11 from the new process stack and update R0*/


	__asm volatile("msr 	psp, r0");			/*Load the value of R0(the new process stack Pointer) into PSP*/
	__asm volatile("isb");						/*Clear Pipe*/

	__asm volatile("bx		lr");				/*return from interrupt with value in LR(0xFFFFFFED)*/
}

//////////////////////////////
/*System Calls*/
//////////////////////////////
int OS_ThreadCreate(const char* name, OS_ThreadHandler_t threadMain, OS_Priority_e priority, void *stack, uint32_t stackSize)
{
	uint32_t* sp;
	OS_Thread_t* thisOsThread;

	if(priority == OS_PRIORITY_NONE)
	{
		thisOsThread = &OS_IdleThread;
	}
	else
	{
		thisOsThread = OS_FindAvailableThread();
		if(!thisOsThread)
		{
			return -1;
		}
	}

	/*Assign the name to the string for it to be referenced*/
	strcpy(thisOsThread->name, name);

	/*Determine the top of the stack*/
	thisOsThread->stack = stack;

	sp = (uint32_t*)((((uint32_t)thisOsThread->stack + stackSize*4 + 1U) / 8) * 8);

	/*Initialise The Stack*/
	*(--sp) = 0x00000000U;
	*(--sp) = (1U << 24);				/*FPSCR*/
	sp -= 0x10;							/*S15 - S0*/

	*(--sp) = (1U << 24);				/*xPSR*/
	*(--sp) = (uint32_t)(threadMain);	/*PC*/
	*(--sp) = (uint32_t)(OS_Done);		/*LR*/
	sp--;								/*R12*/
	sp -= 0x04;							/*R3 - R0*/

	sp -= 0x10;							/*S31- S16*/

	*(--sp) = 0xFFFFFFEDU;				/*LR*/
	sp--;								/*R11*/

	if(priority == OS_PRIORITY_NONE)
	{
		*(--sp) = 0x00000000U;				/*R10*/
	}
	else
	{
		*(--sp) = (uint32_t)(void*)OS_Current->gotTable;/*R10*/
	}

	sp -= 0x06;							/*R9- R4*/

	/*Assign the struct variables*/
	thisOsThread->sp = sp;
	thisOsThread->priority = priority;
	thisOsThread->main = threadMain;
	thisOsThread->active = true;

	if(priority != OS_PRIORITY_NONE)
	{
		thisOsThread->parentThread = OS_Current;
		OS_ReadyMask |= 1U << thisOsThread->id;
	}

	return 0;
}

int OS_ThreadImageCreate(const OS_Image_t *image)
{
	uint32_t* sp;
	OS_Thread_t* thisOsThread;

	thisOsThread = OS_FindAvailableThread();
	if(!thisOsThread)
	{
		return -1;
	}

	/*Assign the name to the string for it to be referenced*/
	strcpy(thisOsThread->name, image->name);

	/*Allocate memory for the GOT Table.*/
	thisOsThread->gotTable = malloc(image->gotSize);

	/*Allocate memory for the ram of the Thread*/
	thisOsThread->ram = malloc((image->ramSize + image->bssSize));

	/*Initialise the Ram from the image*/
	for(uint32_t i = 0; i < image->ramSize; i++)
	{
		thisOsThread->ram[i] = image->ram[i];
	}

	for(uint32_t i = 0; i < image->bssSize; i++)
	{
		thisOsThread->ram[image->ramSize + i] = 0;
	}

	/*Allocate memory for the stack of the thread*/
	thisOsThread->stack = malloc(image->stackSize*sizeof(uint32_t));

	for(uint32_t i = 0; i < image->gotSize/sizeof(uint32_t); i++)
	{
		if(image->got[i] < FLASH_BASE)
		{
			thisOsThread->gotTable[i] = image->got[i] + (uint32_t)(void*)thisOsThread->ram;
		}
		else
		{
			thisOsThread->gotTable[i] = image->got[i];
		}
	}

	/*Determine the top of the stack*/
	sp = (uint32_t*)((((uint32_t)thisOsThread->stack + image->stackSize*4 + 1U) / 8) * 8);

	/*Initialise The Stack*/
	*(--sp) = 0x00000000U;
	*(--sp) = (1U << 24);				/*FPSCR*/
	sp -= 0x10;							/*S15 - S0*/

	*(--sp) = (1U << 24);				/*xPSR*/
	*(--sp) = (uint32_t)(image->main);	/*PC*/
	*(--sp) = (uint32_t)(OS_Done);		/*LR*/
	sp--;								/*R12*/
	sp -= 0x04;							/*R3 - R0*/

	sp -= 0x10;							/*S31- S16*/

	*(--sp) = 0xFFFFFFEDU;				/*LR*/
	sp--;								/*R11*/
	*(--sp) = (uint32_t)(void*)thisOsThread->gotTable;/*R10*/
	sp -= 0x06;							/*R9- R4*/

	/*Assign the struct variables*/
	thisOsThread->sp = sp;
	thisOsThread->priority = image->priority;
	thisOsThread->main = image->main;
	thisOsThread->active = true;

	OS_ReadyMask |= 1U << thisOsThread->id;

	return 0;
}


void OS_Delay(uint32_t delay)
{
	uint32_t bit;
	__disable_irq();

	OS_Current->delay = delay;

	bit = 1U << OS_Current->id;
	OS_ReadyMask &= ~bit;
	OS_DelayMask |= bit;

	OS_Sched();

	__enable_irq();
}

OS_Semaphore_t OS_CreateSemaphore(const char *name, int num, ...)
{
	va_list marker;
	OS_Semaphore_t semaphore;
	uint32_t event;

	semaphore.thread = OS_GetThreadByName(name);
	semaphore.events = 0;

	va_start(marker, num);

	for(int i = 0; i < num; i++)
	{
		event = va_arg(marker, uint32_t);
		semaphore.events |= 1U << event;
	}

	va_end(marker);

	return semaphore;
}

void OS_SetSemaphore(uint32_t event)
{
	OS_Thread_t *waitingThread;
	uint32_t workingMask = OS_Current->waitingThreadMask;

	__disable_irq();

	/*Loop through all the threads that are waiting for an event from this thread.*/
	while(workingMask != 0)
	{
		waitingThread = &OS_Threads[LOG2(workingMask) - 1];
		for(uint8_t i = 0; i < waitingThread->numSemaphores; i++)
		{
			if(waitingThread->semaphores[i].thread->id == OS_Current->id)
			{
				if((1U << event) & waitingThread->semaphores[i].events)
				{
					waitingThread->semaphores[i].triggered = event;
					waitingThread->semaphores = &waitingThread->semaphores[i];
					OS_SetThreadActive(waitingThread);
				}
				break;
			}
		}
		workingMask &= ~(1U << waitingThread->id);
	}

	__enable_irq();
}

void OS_WaitSemaphores(uint8_t num, OS_Semaphore_t *semaphores, uint32_t timeout)
{
	__disable_irq();

	OS_Current->semaphores = semaphores;
	OS_Current->numSemaphores = num;

	if(timeout != OS_NO_TIMEOUT)
	{
		OS_Current->delay = timeout;
		OS_DelayMask |= 1U << OS_Current->id;
	}

	OS_Current->threadMask = 0;
	for(uint8_t i = 0; i < num; i++)
	{
		semaphores[i].triggered = OS_NO_EVENT;
		semaphores[i].thread->waitingThreadMask |= 1U << OS_Current->id;
		OS_Current->threadMask |= (1U << semaphores[i].thread->id);
	}

	/*Clear the ready mask of this thread.*/
	OS_ReadyMask &= ~(1U << OS_Current->id);

	/*Call the scheduler.*/
	OS_Sched();

	__enable_irq();
}

OS_Thread_t* OS_GetThreadByName(const char *name)
{
	uint32_t threadTrack = 0;
	while((strcmp(OS_Threads[threadTrack].name, name) != 0) && (threadTrack < OS_MAX_THREADS))
	{
		threadTrack++;
	}

	if(threadTrack < OS_MAX_THREADS)
	{
		return &OS_Threads[threadTrack];
	}

	return NULL;
}

bool OS_TriggeredThread(const char *name)
{
	if(strcmp(OS_Current->semaphores->thread->name, name) == 0)
	{
		return true;
	}

	return false;
}

uint8_t OS_TriggeredEvent(void)
{
	return OS_Current->semaphores->triggered;
}

void OS_SetThreadData(void *data)
{
	OS_Current->channelData = data;
}

void *OS_GetThreadData(void)
{
	return OS_Current->semaphores->thread->channelData;
}

void OS_SetInterrupt(OS_ThreadHandler_t interruptHandler, IRQn_Type interrupt)
{
	OS_VectorTable[interrupt + 16] = (uint32_t)interruptHandler;
}

void OS_KillThread(const char *name)
{
	OS_Thread_t* thisOsThread = OS_GetThreadByName(name);

	if(thisOsThread->parentThread != OS_Current)
	{
		return;
	}

	thisOsThread->active = false;

	if(thisOsThread->ram)
		free(thisOsThread->ram);
	if(thisOsThread->gotTable)
		free(thisOsThread->gotTable);
	if(thisOsThread->stack)
		free(thisOsThread->stack);

	OS_ReadyMask &= ~(1U << thisOsThread->id);
	OS_DelayMask &= ~(1U << thisOsThread->id);
}

void OS_Kill(void)
{
	OS_Current->active = false;

	if(OS_Current->ram)
		free(OS_Current->ram);
	if(OS_Current->gotTable)
		free(OS_Current->gotTable);
	if(OS_Current->stack)
		free(OS_Current->stack);

	OS_ReadyMask &= ~(1U << OS_Current->id);
	OS_DelayMask &= ~(1U << OS_Current->id);

	OS_Sched();
}
