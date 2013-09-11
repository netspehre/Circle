/* use setjmp/longjmp to implement coroutines */
#define USE_STDPERIPH_DRIVER
//#include <stdio.h>
//#include <stdlib.h>
#include "stm32f4xx_it.h"
#include "main.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "stm32f4_discovery.h"
#include "usbd_hid_core.h"
#include <setjmp.h>
//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"

#define STACK_SIZE 4096

/*
 * Change SP prior to calling setjmp so that longjmp will
 * start the routine with 'stackptr'.
 */

#define SAVE_STACK_POINTER_ASM(savedstack, stackptr) \
	"mov r15, %[savedstack]\n" /* savedstack <- SP */ \
	"mov %[stackptr], r15"    /* SP <- stackptr */

#define SAVE_STACK_POINTER(savedstack, stackptr) \
do { \
	asm volatile ( SAVE_STACK_POINTER_ASM(savedstack, stackptr) \
	: [savedstack] "=r" (savedstack): [stackptr] "r" (stackptr) ); \
} while (0)

/* Restore "normal" stack prior to return */

#define RESTORE_STACK_ASM(savedstack) \
	"mov %[savedstack],r15"

#define RESTORE_STACK(savedstack) \
do { \
	asm volatile ( RESTORE_STACK_ASM(savedstack) \
	: : [savedstack] "r" (savedstack)); \
} while (0)

jmp_buf routine1_buf;
jmp_buf routine2_buf;

const char MyTable[180][2]= {{100,0},{100,3},{100,7},{99,10},{99,14},{98,17},{98,21},{97,24},{96,28},{95,31},{94,34},{93,37},{91,41},{90,44},{88,47},{87,50},{85,53},{83,56},{81,59},{79,62},{77,64},{74,67},{72,69},{69,72},{67,74},{64,77},{62,79},{59,81},{56,83},{53,85},{50,87},{47,88},{44,90},{41,91},{37,93},{34,94},{31,95},{28,96},{24,97},{21,98},{17,98},{14,99},{10,99},{7,100},{3,100},{0,100},{-3,100},{-7,100},{-10,99},{-14,99},{-17,98},{-21,98},{-24,97},{-28,96},{-31,95},{-34,94},{-37,93},{-41,91},{-44,90},{-47,88},{-50,87},{-53,85},{-56,83},{-59,81},{-62,79},{-64,77},{-67,74},{-69,72},{-72,69},{-74,67},{-77,64},{-79,62},{-81,59},{-83,56},{-85,53},{-87,50},{-88,47},{-90,44},{-91,41},{-93,37},{-94,34},{-95,31},{-96,28},{-97,24},{-98,21},{-98,17},{-99,14},{-99,10},{-100,7},{-100,3},{-100,0},{-100,-3},{-100,-7},{-99,-10},{-99,-14},{-98,-17},{-98,-21},{-97,-24},{-96,-28},{-95,-31},{-94,-34},{-93,-37},{-91,-41},{-90,-44},{-88,-47},{-87,-50},{-85,-53},{-83,-56},{-81,-59},{-79,-62},{-77,-64},{-74,-67},{-72,-69},{-69,-72},{-67,-74},{-64,-77},{-62,-79},{-59,-81},{-56,-83},{-53,-85},{-50,-87},{-47,-88},{-44,-90},{-41,-91},{-37,-93},{-34,-94},{-31,-95},{-28,-96},{-24,-97},{-21,-98},{-17,-98},{-14,-99},{-10,-99},{-7,-100},{-3,-100},{-0,-100},{3,-100},{7,-100},{10,-99},{14,-99},{17,-98},{21,-98},{24,-97},{28,-96},{31,-95},{34,-94},{37,-93},{41,-91},{44,-90},{47,-88},{50,-87},{53,-85},{56,-83},{59,-81},{62,-79},{64,-77},{67,-74},{69,-72},{72,-69},{74,-67},{77,-64},{79,-62},{81,-59},{83,-56},{85,-53},{87,-50},{88,-47},{90,-44},{91,-41},{93,-37},{94,-34},{95,-31},{96,-28},{97,-24},{98,-21},{98,-17},{99,-14},{99,-10},{100,-7},{100,-3}};

extern int MyN;
extern int MyBrightness[4];

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
int Phase=0;
uint8_t HID_Buffer[4] = {1};

char MyStack1[STACK_SIZE];
char MyStack2[STACK_SIZE];

void MyDelay()
{
    volatile int i;
    for(i=0; i<100000; i++);
}

void LED_AdjustPWN(void)
{
    setjmp(routine1_buf);
    switch (Phase)
    {
    case 0 ... 44:
        MyBrightness[3] -= MyN;
        MyBrightness[0] += MyN;
        break;
    case 45 ... 89:
        MyBrightness[0] -= MyN;
        MyBrightness[1] += MyN;
        break;
    case 90 ... 134:
        MyBrightness[1] -= MyN;
        MyBrightness[2] += MyN;
        break;
    case 135 ... 179:
        MyBrightness[2] -= MyN;
        MyBrightness[3] += MyN;
        break;
    }
    TIM4->CCR1 = MyBrightness[0];
    TIM4->CCR2 = MyBrightness[1];
    TIM4->CCR3 = MyBrightness[2];
    TIM4->CCR4 = MyBrightness[3];
    MyDelay();
    longjmp(routine2_buf,1);
}
 
void USBD_HID_GetPos(void)
{
    static int CurrentX=100;
    static int CurrentY=0;
    static int NextX=0;
    static int NextY=0;
    setjmp(routine2_buf);
    NextX=MyTable[Phase][0];
    NextY=MyTable[Phase][1];
    HID_Buffer[1]=NextX-CurrentX;
    HID_Buffer[2]=NextY-CurrentY;
    CurrentX=NextX;
    CurrentY=NextY;
    Phase=(Phase+1)%180;
    USBD_HID_SendReport (&USB_OTG_dev, HID_Buffer, 4);
    MyDelay();
    longjmp(routine1_buf,1);
 
}

void create_routine1(const void *stackptr)
{
        
	register unsigned long savedstack;
	SAVE_STACK_POINTER(savedstack, stackptr);
        
	if (setjmp(routine1_buf) == 0) {
                
		RESTORE_STACK(savedstack);
	}
	else {
		/* We got here through longjmp */
                
		LED_AdjustPWN();
	}
        
}

void create_routine2(const void *stackptr)
{
        //TIM4->CCR2 = 3000;
	register unsigned long savedstack;

	SAVE_STACK_POINTER(savedstack, stackptr);

	if (setjmp(routine2_buf) == 0) {
		RESTORE_STACK(savedstack);
	}
	else {
		USBD_HID_GetPos();
	}
}

int InitializeCoroutine(void)
{
        
	create_routine1(MyStack1 + STACK_SIZE);
	create_routine2(MyStack2 + STACK_SIZE);
        
	longjmp(routine1_buf, 1);
	return 0;
}
