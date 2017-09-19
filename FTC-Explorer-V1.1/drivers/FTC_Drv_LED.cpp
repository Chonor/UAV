/******************** (C) COPYRIGHT 2015 FTC ***************************
 * 作者		 ：FTC
 * 文件名  ：FTC_Drv_LED.cpp
 * 描述    ：LED
**********************************************************************************/
#include "FTC_Drv_LED.h"

FTC_LED led;

void FTC_LED::Init(void)
{
	//return;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	
	//EN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = (FTC_Pin_LED1 | FTC_Pin_LED2);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	//To do
}

void FTC_LED::ON1(void)
{
	//return;
	GPIO_SetBits(FTC_GPIO_LED1, FTC_Pin_LED1);
	//To do			
}

void FTC_LED::ON2(void)
{		
	//return;
	GPIO_SetBits(FTC_GPIO_LED2, FTC_Pin_LED2);
	//To do	
}

void FTC_LED::OFF1(void)
{
	//return;
	GPIO_ResetBits(FTC_GPIO_LED1, FTC_Pin_LED1);
	//To do
}

void FTC_LED::OFF2(void)
{
	//return;
	GPIO_ResetBits(FTC_GPIO_LED2, FTC_Pin_LED2);
	//To do	
}


/******************* (C) COPYRIGHT 2015 FTC *****END OF FILE************/

