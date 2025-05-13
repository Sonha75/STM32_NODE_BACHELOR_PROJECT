/*
 * myLibrary.h
 *
 *  Created on: Dec 20, 2020
 *      Author: vikto
 */

#ifndef MULTI_HCSR04_H_
#define MULTI_HCSR04_H_

#include "main.h"

extern TIM_HandleTypeDef htim2;

#define TriggerDuration 2

extern uint16_t distance, triggerTime, sensor;
extern GPIO_TypeDef *triggerPorts[2];
extern uint16_t triggerPins[2];
extern GPIO_TypeDef *echoPorts[2];
extern uint16_t echoPins[2];

void SysTickEnable();
void SysTickDisable();
uint16_t measureDistance(GPIO_TypeDef *triggerPort, uint16_t triggerPin, GPIO_TypeDef *echoPort, uint16_t echoPin);

#endif
