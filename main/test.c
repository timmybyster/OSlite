/*
 * test.c
 *
 *  Created on: 24 Feb 2020
 *      Author: Tim Buckley
 */
#include "../Src/os/os.h"

#define EVENT_0		0
#define EVENT_1		1

const char mainName[] = "MAIN";

extern char mainData asm("_mainsidata");
extern char mainDataSize asm("_main_data_size");
extern char mainBssSize asm("_main_bss_size");
extern char mainGotData asm("_got");
extern char mainGotSize asm("_main_got_size");

void mainThread(void);

const OS_Image_t __attribute__((__section__(".mainImageSection")))testImage1 = {
		.got = (void*)&mainGotData,
		.gotSize = (uint32_t)&mainGotSize,
		.main = mainThread,
		.name = mainName,
		.priority = OS_PRIORITY_HIGH,
		.ram = (void*)&mainData,
		.ramSize = (uint32_t)&mainDataSize,
		.bssSize = (uint32_t)&mainBssSize,
		.stackSize = 100
};

uint8_t i = 0;
uint32_t thisIsATestVariable = 9;

uint32_t tempStack[100];

void testCall(void)
{
	i++;
}

void interruptRoutine(void)
{

}

void testThreadMain(void)
{
	uint8_t test = 0;
	while(1)
	{
		OS_Delay(100);
		if(test == 1)
		{
			OS_SetSemaphore(EVENT_0);
		}
	}
}

void mainThread(void)
{

	OS_SetInterrupt(interruptRoutine, SysTick_IRQn);

	OS_ThreadCreate("THREAD", testThreadMain, OS_PRIORITY_HIGH, tempStack, 100);

	OS_Semaphore_t semaphore[1] = {OS_CreateSemaphore("THREAD", 2, EVENT_0, EVENT_1)};

	uint8_t* ptr = calloc(10, 1);

	testCall();

	while(1)
	{
		OS_WaitSemaphores(1, semaphore, OS_NO_TIMEOUT);
		ptr[thisIsATestVariable] = i;
		thisIsATestVariable++;
		if(thisIsATestVariable == 10)
		{
			thisIsATestVariable = 0;
		}
		i++;
		OS_KillThread("THREAD");
		OS_Delay(100);
	}
}
