/*
 * os.h
 *
 *  Created on: Feb 12, 2020
 *      Author: Tim Buckley
 */

#ifndef OS_OS_H_
#define OS_OS_H_

#include "main.h"
#include "string.h"
#include "stddef.h"
#include "stdbool.h"
#include "malloc.h"
#include "stdarg.h"
#include <errno.h>
#include <stdio.h>

#define OS_MAX_THREADS					32
#define OS_MIN_STACK_SIZE				100
#define OS_MAX_NAME_SIZE				10
#define OS_NO_EVENT						0xFFFFFFFFUL
#define OS_NO_TIMEOUT					0xFFFFFFFFUL
#define OS_NO_THREAD_ID					0xFF
#define OS_VECTOR_TABLE_SIZE			100UL
#define OS_VECTORTABLE_ALIGNMENT   		0x100UL
#define	OS_APPLICATION_RAM				0x20001800

typedef void (*OS_ThreadHandler_t)();

typedef enum
{
	OS_PRIORITY_NONE 	= 0,
	OS_PRIORITY_LOW		= 10,
	OS_PRIORITY_MEDIUM  = 100,
	OS_PRIORITY_HIGH	= 1000,
}OS_Priority_e;

typedef enum
{
	OS_STATE_NONE		= 0,
	OS_STATE_BLOCKED,
	OS_STATE_READY,
	OS_STATE_ACTIVE,
	OS_STATE_ZOMBIE
}OS_State_e;

struct OS_Semaphore;

typedef struct OS_Thread
{
	void* sp;					/*Stack Pointer*/
	void* stack;				/*Stack Pointer*/
	uint8_t* ram;				/*Ram pointer*/
	uint32_t *gotTable;			/*The address of the GOT table offset in Ram*/
	OS_ThreadHandler_t main;	/*Main Function to execute*/
	OS_Priority_e priority;		/*Priority of the thread*/
	OS_State_e state;			/*Process Current*/
	uint32_t startAddress;		/*Address where image is located*/
	uint8_t id;					/*Thread ID*/
	bool active;				/*IS the thread memory available to be used.*/
	uint32_t readySkips;		/*Counter for thread not executed when Ready*/
	char name[OS_MAX_NAME_SIZE];
	uint32_t waitingThreadMask;	/*Threads blocked by this thread*/
	uint32_t threadMask;		/*Threads Blocking this thread*/
	uint32_t delay;				/*Delay wait counter*/
	struct OS_Semaphore* semaphores;
	void* channelData;
	uint8_t numSemaphores;
	struct OS_Thread* parentThread;
}OS_Thread_t;

typedef struct OS_Semaphore
{
	OS_Thread_t* thread;
	uint64_t events;
	uint32_t triggered;
}OS_Semaphore_t;

typedef struct
{
	OS_ThreadHandler_t main;	/*Main Function to execute*/
	uint32_t* got;
	uint32_t gotSize;
	uint8_t* ram;
	uint32_t ramSize;
	uint32_t bssSize;
	const char* name;
	uint32_t stackSize;
	OS_Priority_e priority;
}OS_Image_t;

void OS_Init(TIM_HandleTypeDef*);
void OS_Run(void);

int OS_ThreadCreate(const char*, OS_ThreadHandler_t, OS_Priority_e, void*, uint32_t);

void OS_Delay(uint32_t);

OS_Semaphore_t OS_CreateSemaphore(const char*, int, ...);
void OS_SetSemaphore(uint32_t);
void OS_WaitSemaphores(uint8_t, OS_Semaphore_t*, uint32_t);

OS_Thread_t* OS_GetThreadByName(const char*);

bool OS_TriggeredThread(const char*);
uint8_t OS_TriggeredEvent(void);

void OS_SetThreadData(void*);
void *OS_GetThreadData(void);

void OS_SetInterrupt(OS_ThreadHandler_t, IRQn_Type);

void OS_KillThread(const char*);
void OS_Kill(void);


#endif /* OS_OS_H_ */
