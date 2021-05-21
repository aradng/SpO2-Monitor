/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2007
 *
 *    File name   : main.c
 *    Description : Define main module
 *
 *    History :
 *    1. Date        : 19, July 2006
 *       Author      : Stanimir Bonev
 *       Description : Create
 *
 *  This example project shows how to use the IAR Embedded Workbench for ARM
 * to develop code for the IAR-STM32-SK evaluation board. It shows
 * basic use of I/O, timer and the interrupt controllers.
 *  Displays running lights on the board LED's.
 *
 *  Jumpers:
 *   PWR_SEL - depending of power source
 *
 *    $Revision: 39 $
 **************************************************************************/
#include "main.h"
#include <stdio.h>
#include <intrinsics.h>
#include "stm32f10x.h"
#include "dac_sw/dac_sw.h"
#include "ssd1306/ssd1306.h"

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

uint32_t itt = 0;
uint16_t data[3][10000];
extern uint32_t millis(void);
/*************************************************************************
 * Function Name: Timer1IntrHandler
 * Parameters: none
 *
 * Return: none
 *
 * Description: Timer 1 interrupt handler
 *
 *************************************************************************/
void Timer1IntrHandler(void)
{
  // Clear update interrupt bit
  TIM_ClearITPendingBit(TIM1,TIM_FLAG_Update);
  
  /*printf(%i/r/n , adc_read(1));
  //data[itt % 3][itt / 3]  = adc_read(0);
  dac1.value = 0x0fff * (itt + 0 % 3);
  dac2.value = 0x0fff * (itt + 1 % 3);
  dac3.value = 0x0fff * (itt + 2 % 3);
  dac1.update();
  dac2.update();
  dac3.update();
*/
  itt++;
  if(itt >= 30000)                    //ble data
  {
    /*printf("/r/nR:/r/n");
      for(int i = 0 ; i < itt/3 ; i++)
        printf("%i ", data[0][i]);
    printf("/r/nIR:/r/n");
      for(int i = 0 ; i < itt/3 ; i++)
        printf("%i ", data[0][i]);
    printf("/r/nG:/r/n");
      for(int i = 0 ; i < itt/3 ; i++)
        printf("%i ", data[0][i]);*/
  itt = 0;
  }
}

void usart_init(void);
void adc_init(void);
uint16_t adc_read(bool wait);

void main(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM1_TimeBaseInitStruct;

  __disable_interrupt();

  /* Setup STM32 system (clock, PLL and Flash configuration) */
  SystemInit();

  // NVIC init
#ifndef  EMB_FLASH
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  // GPIO Init
  // Enable GPIO clock and release reset
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC,
                         ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC,
                         DISABLE);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 |
                                GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_12|
                                GPIO_Pin_10| GPIO_Pin_11| GPIO_Pin_13|
                                GPIO_Pin_14| GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  // Timer1 Init
  // Enable Timer1 clock and release reset
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1,DISABLE);

  // Set timer period 0.1 sec
  TIM1_TimeBaseInitStruct.TIM_Prescaler = 720;  // 10us resolution
  TIM1_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM1_TimeBaseInitStruct.TIM_Period = 10000;  // 100 ms
  TIM1_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM1_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1,&TIM1_TimeBaseInitStruct);

  // Clear update interrupt bit
  TIM_ClearITPendingBit(TIM1,TIM_FLAG_Update);
  // Enable update interrupt
  TIM_ITConfig(TIM1,TIM_FLAG_Update,ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable timer counting
  TIM_Cmd(TIM1,ENABLE);

  __enable_interrupt();
  
  // SETUP
  adc_init();
  
  dac dac1(DAC_1_GPIO_Port ,DAC_1_Pin ,SCK_GPIO_Port ,SCK_Pin ,SDI_GPIO_Port ,SDI_Pin);
  dac dac2(DAC_1_GPIO_Port ,DAC_1_Pin ,SCK_GPIO_Port ,SCK_Pin ,SDI_GPIO_Port ,SDI_Pin);
  dac dac3(DAC_1_GPIO_Port ,DAC_1_Pin ,SCK_GPIO_Port ,SCK_Pin ,SDI_GPIO_Port ,SDI_Pin);

  dac1.value = 0x0fff;
  dac2.value = 0x0fff;
  dac3.value = 0x0fff;
  dac1.update();
  dac2.update();
  dac3.update();
  
  while(1)
  {
    printf("%i/r/n" , millis());
  }
}
  
void adc_init()
{
  ADC_InitTypeDef hadc1;
  hadc1.ADC_Mode                = ADC_Mode_Independent;
  hadc1.ADC_ScanConvMode        = DISABLE;
  hadc1.ADC_ContinuousConvMode  = DISABLE;
  hadc1.ADC_ExternalTrigConv    = ADC_ExternalTrigInjecConv_None;
  hadc1.ADC_DataAlign           = ADC_DataAlign_Right;
  hadc1.ADC_NbrOfChannel        = 1;
  
  ADC_Init(ADC1 ,&hadc1);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
  ADC_TempSensorVrefintCmd(ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);  
  while(ADC_GetCalibrationStatus(ADC1));
}

uint16_t adc_read(bool wait)
{
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while((!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))*wait);
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  uint16_t AD_value = ADC_GetConversionValue(ADC1);
  return AD_value;
}

void usart1_init(void)
{
  USART_InitTypeDef huart1;
  huart1.USART_BaudRate                 = 115200;
  huart1.USART_WordLength               = USART_WordLength_8b;
  huart1.USART_StopBits                 = USART_StopBits_1;
  huart1.USART_Parity                   = USART_Parity_No;
  huart1.USART_Mode                     = USART_Mode_Rx | USART_Mode_Tx;
  huart1.USART_HardwareFlowControl      = USART_HardwareFlowControl_None;
  
  USART_Init(USART1 ,&huart1);
  USART_Cmd(USART1, ENABLE);
}

void usart2_init(void)
{
  USART_InitTypeDef huart2;
  huart2.USART_BaudRate                 = 115200;
  huart2.USART_WordLength               = USART_WordLength_8b;
  huart2.USART_StopBits                 = USART_StopBits_1;
  huart2.USART_Parity                   = USART_Parity_No;
  huart2.USART_Mode                     = USART_Mode_Rx | USART_Mode_Tx;
  huart2.USART_HardwareFlowControl      = USART_HardwareFlowControl_None;
  
  USART_Init(USART2 ,&huart2);
  USART_Cmd(USART2, ENABLE);
}

int _write(int fd, char * ptr, int len)                                                                         //not sure                      test putchar/write with full std lib
{
  int tlen = len;
  //if(fd == 0)                                                                                                 //fd 0 : stdin || 1 : stdout  || 2 : stderr
    while(tlen > 0)
    {
      USART_SendData(USART2, *((uint8_t *) ptr++));
      while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
      tlen--;
    }
  return len;
}
PUTCHAR_PROTOTYPE
{
  USART_SendData(USART2, (uint8_t)ch);
  return ch;
}