/*
 * main2.c
 *
 *  Created on: Feb 28, 2020
 *      Author: Tim Buckley
 */

#include "../Src/os/os.h"

const char mainName2[] = "MAIN2";

extern char main2Data asm("_main2sidata");
extern char main2DataSize asm("_main2_data_size");
extern char main2BssSize asm("_main2_bss_size");
extern char main2GotData asm("_got");
extern char main2GotSize asm("_main2_got_size");

void main2Thread(void);

const OS_Image_t __attribute__((__section__(".main2ImageSection")))main2Image = {
		.got = (void*)&main2GotData,
		.gotSize = (uint32_t)&main2GotSize,
		.main = main2Thread,
		.name = mainName2,
		.priority = OS_PRIORITY_HIGH,
		.ram = (void*)&main2Data,
		.ramSize = (uint32_t)&main2DataSize,
		.bssSize = (uint32_t)&main2BssSize,
		.stackSize = 100
};

void main2Thread(void)
{
	while(1)
	{

	}
}
