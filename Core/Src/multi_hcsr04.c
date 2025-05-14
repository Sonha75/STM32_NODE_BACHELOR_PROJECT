

#include <multi_hcsr04.h>


uint16_t distance = 0, triggerTime = 0, sensor = 0;
GPIO_TypeDef *triggerPorts[2] = {TRIG1_GPIO_Port, TRIG2_GPIO_Port};
uint16_t triggerPins[2] = {TRIG1_Pin, TRIG2_Pin};
GPIO_TypeDef *echoPorts[2] = {ECHO1_GPIO_Port, ECHO2_GPIO_Port};
uint16_t echoPins[2] = {ECHO1_Pin, ECHO2_Pin};

void SysTickEnable()
{
	__disable_irq();
	SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
	__enable_irq();
}

void SysTickDisable()
{
	__disable_irq();
	SysTick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
	__enable_irq();
}

uint16_t measureDistance(GPIO_TypeDef *triggerPort, uint16_t triggerPin, GPIO_TypeDef *echoPort, uint16_t echoPin)
{
	if(!HAL_GPIO_ReadPin(echoPort, echoPin))//skip sensor if ECHO pin is still busy
	{
		SysTickDisable();
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_SET);
		triggerTime = 0;//reset the variable
		asm ("nop");//to avoid program freezing
		while(triggerTime < TriggerDuration);
		HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_RESET);
		while(!HAL_GPIO_ReadPin(echoPort, echoPin));
		distance = 0;//reset the variable
		while(HAL_GPIO_ReadPin(echoPort, echoPin));
		HAL_TIM_Base_Stop_IT(&htim2);
		SysTickEnable();
	}else//give max distance if ECHO pin is still busy
	{
		distance = 500;
	}
	return distance;
}
