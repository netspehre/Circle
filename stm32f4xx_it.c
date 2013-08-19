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
__IO uint8_t TempAcceleration = 0;
//int8_t step[ 4 ][ 2 ] = { { 7, 0 }, { 0, 7 }, { -7, 0 }, { 0, -7 } };
uint8_t step[ 4 ][ 2 ] = { { 0, 7 }, { 7, 0 }, { 0, -7 }, { -7, 0 } };
uint8_t StepCnt = 0, CurDir = 0;
const char MyTable[180][2]={{100,0},{100,3},{100,7},{99,10},{99,14},{98,17},{98,21},{97,24},{96,28},{95,31},{94,34},{93,37},{91,41},{90,44},{88,47},{87,50},{85,53},{83,56},{81,59},{79,62},{77,64},{74,67},{72,69},{69,72},{67,74},{64,77},{62,79},{59,81},{56,83},{53,85},{50,87},{47,88},{44,90},{41,91},{37,93},{34,94},{31,95},{28,96},{24,97},{21,98},{17,98},{14,99},{10,99},{7,100},{3,100},{0,100},{-3,100},{-7,100},{-10,99},{-14,99},{-17,98},{-21,98},{-24,97},{-28,96},{-31,95},{-34,94},{-37,93},{-41,91},{-44,90},{-47,88},{-50,87},{-53,85},{-56,83},{-59,81},{-62,79},{-64,77},{-67,74},{-69,72},{-72,69},{-74,67},{-77,64},{-79,62},{-81,59},{-83,56},{-85,53},{-87,50},{-88,47},{-90,44},{-91,41},{-93,37},{-94,34},{-95,31},{-96,28},{-97,24},{-98,21},{-98,17},{-99,14},{-99,10},{-100,7},{-100,3},{-100,0},{-100,-3},{-100,-7},{-99,-10},{-99,-14},{-98,-17},{-98,-21},{-97,-24},{-96,-28},{-95,-31},{-94,-34},{-93,-37},{-91,-41},{-90,-44},{-88,-47},{-87,-50},{-85,-53},{-83,-56},{-81,-59},{-79,-62},{-77,-64},{-74,-67},{-72,-69},{-69,-72},{-67,-74},{-64,-77},{-62,-79},{-59,-81},{-56,-83},{-53,-85},{-50,-87},{-47,-88},{-44,-90},{-41,-91},{-37,-93},{-34,-94},{-31,-95},{-28,-96},{-24,-97},{-21,-98},{-17,-98},{-14,-99},{-10,-99},{-7,-100},{-3,-100},{-0,-100},{3,-100},{7,-100},{10,-99},{14,-99},{17,-98},{21,-98},{24,-97},{28,-96},{31,-95},{34,-94},{37,-93},{41,-91},{44,-90},{47,-88},{50,-87},{53,-85},{56,-83},{59,-81},{62,-79},{64,-77},{67,-74},{69,-72},{72,-69},{74,-67},{77,-64},{79,-62},{81,-59},{83,-56},{85,-53},{87,-50},{88,-47},{90,-44},{91,-41},{93,-37},{94,-34},{95,-31},{96,-28},{97,-24},{98,-21},{98,-17},{99,-14},{99,-10},{100,-7},{100,-3}};
/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
static uint8_t *USBD_HID_GetPos (void);
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
void HID_Release();
unsigned int Real_Sqrt(unsigned long int M);
 
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
    uint8_t *buf;
 
    if ( !DemoEnterCondition )
        return;
 
    buf = USBD_HID_GetPos();                              /*the critical function*/
    USBD_HID_SendReport (&USB_OTG_dev, buf, 4);
 
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
 
/**
* @brief  USBD_HID_GetPos
* @param  None
* @retval Pointer to report
*/
static uint8_t *USBD_HID_GetPos (void)
{
    static int Phase=1;
    static int MyX=100;
    static int MyY=0;
    static int NextX=0;
    static int NextY=0;
    static uint8_t HID_Buffer[4] = {0};

    
    NextX=MyTable[Phase][0];
    NextY=MyTable[Phase][1];
    HID_Buffer[0]=1;
    HID_Buffer[1]=NextX-MyX;
    HID_Buffer[2]=NextY-MyY; 
    MyX=NextX;
    MyY=NextY;
    Phase=(Phase+1)%180;
    return HID_Buffer;
}
 
void HID_Release()
{
    uint8_t HID_Buffer[4] = {0};
    USBD_HID_SendReport (&USB_OTG_dev, HID_Buffer, 4);
}
 
 
unsigned int Real_Sqrt(unsigned long int M)
{
    unsigned int N, i;
    unsigned long tmp, ttp;  
    if (M == 0)               
        return 0;
    N = 0;
    tmp = (M >> 30);         
    M <<= 2;
    if (tmp > 1)              
    {
        N ++;                 
        tmp -= N;
    }
    for (i=15; i>0; i--)      
    {
        N <<= 1;             
        tmp <<= 2;
        tmp += (M >> 30);     
        ttp = N;
        ttp = (ttp<<1)+1;
        M <<= 2;
        if (tmp >= ttp)       
        {
            tmp -= ttp;
            N ++;
        }
    }
 
    return N;
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
 
 
