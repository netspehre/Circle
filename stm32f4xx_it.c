/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "stm32f4_discovery.h"
#include "usbd_hid_core.h"
#include <math.h>
#include <setjmp.h>
//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CURSOR_STEP     7
#define WIDTH	10

extern uint8_t Buffer[6];
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint8_t DemoEnterCondition;
uint8_t Counter  = 0x00;
extern int8_t X_Offset;
extern int8_t Y_Offset;
extern __IO uint8_t UserButtonPressed;

extern int MyN;
extern int MyBrightness[4];
__IO uint8_t TempAcceleration = 0;
const char MyTable[180][2]= {{100,0},{100,3},{100,7},{99,10},{99,14},{98,17},{98,21},{97,24},{96,28},{95,31},{94,34},{93,37},{91,41},{90,44},{88,47},{87,50},{85,53},{83,56},{81,59},{79,62},{77,64},{74,67},{72,69},{69,72},{67,74},{64,77},{62,79},{59,81},{56,83},{53,85},{50,87},{47,88},{44,90},{41,91},{37,93},{34,94},{31,95},{28,96},{24,97},{21,98},{17,98},{14,99},{10,99},{7,100},{3,100},{0,100},{-3,100},{-7,100},{-10,99},{-14,99},{-17,98},{-21,98},{-24,97},{-28,96},{-31,95},{-34,94},{-37,93},{-41,91},{-44,90},{-47,88},{-50,87},{-53,85},{-56,83},{-59,81},{-62,79},{-64,77},{-67,74},{-69,72},{-72,69},{-74,67},{-77,64},{-79,62},{-81,59},{-83,56},{-85,53},{-87,50},{-88,47},{-90,44},{-91,41},{-93,37},{-94,34},{-95,31},{-96,28},{-97,24},{-98,21},{-98,17},{-99,14},{-99,10},{-100,7},{-100,3},{-100,0},{-100,-3},{-100,-7},{-99,-10},{-99,-14},{-98,-17},{-98,-21},{-97,-24},{-96,-28},{-95,-31},{-94,-34},{-93,-37},{-91,-41},{-90,-44},{-88,-47},{-87,-50},{-85,-53},{-83,-56},{-81,-59},{-79,-62},{-77,-64},{-74,-67},{-72,-69},{-69,-72},{-67,-74},{-64,-77},{-62,-79},{-59,-81},{-56,-83},{-53,-85},{-50,-87},{-47,-88},{-44,-90},{-41,-91},{-37,-93},{-34,-94},{-31,-95},{-28,-96},{-24,-97},{-21,-98},{-17,-98},{-14,-99},{-10,-99},{-7,-100},{-3,-100},{-0,-100},{3,-100},{7,-100},{10,-99},{14,-99},{17,-98},{21,-98},{24,-97},{28,-96},{31,-95},{34,-94},{37,-93},{41,-91},{44,-90},{47,-88},{50,-87},{53,-85},{56,-83},{59,-81},{62,-79},{64,-77},{67,-74},{69,-72},{72,-69},{74,-67},{77,-64},{79,-62},{81,-59},{83,-56},{85,-53},{87,-50},{88,-47},{90,-44},{91,-41},{93,-37},{94,-34},{95,-31},{96,-28},{97,-24},{98,-21},{98,-17},{99,-14},{99,-10},{100,-7},{100,-3}};


/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
void USBD_HID_GetPos (void);
void LED_AdjustPWN(void);
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

int Phase=0;
uint8_t HID_Buffer[4] = {1};
extern jmp_buf MyEnv[3];


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}


/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    /*
        if ( !DemoEnterCondition )
            return;
    */

}

/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
    DemoEnterCondition = 0x01;  /*Here*/

    /* Clear the EXTI line pending bit */
    EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}

/**
  * @brief  This function handles EXTI15_10_IRQ Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_WKUP_IRQHandler(void)
{
    if(USB_OTG_dev.cfg.low_power)
    {
        /* Reset SLEEPDEEP and SLEEPONEXIT bits */
        SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));

        /* After wake-up from sleep mode, reconfigure the system clock */
        SystemInit();
        USB_OTG_UngateClock(&USB_OTG_dev);
    }
    EXTI_ClearITPendingBit(EXTI_Line18);
}
 
/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
    USBD_OTG_ISR_Handler (&USB_OTG_dev);
}
 
void MyDelay()
{
    volatile int i;
    for(i=0; i<100000; i++);
}
 
void Switcher()
{
 
    static int A=0;
    LED_AdjustPWN();
    setjmp(MyEnv[3]);
    MyDelay();
    if ( A == 0 )
    {
        A=1;
        longjmp(MyEnv[0],1);
    }
    else
    {
        A=0;
        longjmp(MyEnv[1],1);
    }
}
 
/**
* @brief  USBD_HID_GetPos
* @param  None
* @retval Pointer to report
*/
 
void LED_AdjustPWN(void)
{
    if (0==setjmp(MyEnv[0]))
    {
        USBD_HID_GetPos();
        return;
    }
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
 
    longjmp(MyEnv[3],1);
}
 
void USBD_HID_GetPos(void)
{
    static int CurrentX=100;
    static int CurrentY=0;
    static int NextX=0;
    static int NextY=0;
    if (0==setjmp(MyEnv[1]))
    {
        return;
    }
    NextX=MyTable[Phase][0];
    NextY=MyTable[Phase][1];
    HID_Buffer[1]=NextX-CurrentX;
    HID_Buffer[2]=NextY-CurrentY;
    CurrentX=NextX;
    CurrentY=NextY;
    Phase=(Phase+1)%180;
    USBD_HID_SendReport (&USB_OTG_dev, HID_Buffer, 4);
 
    longjmp(MyEnv[3],1);
 
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
 
 
 
