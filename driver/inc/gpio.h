/*
 * gpio.h
 *
 *  Created on: Oct 6, 2023
 *      Author: ADMIN
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include <stm32f103.h>
#include <stdbool.h>

typedef struct
{
	GPIO_TypeDef *portBase;
	PORT_t 	port;
	uint8_t pinNumber;
	uint8_t pinMode;
	uint8_t pinSpeed;
	uint8_t pullRes;
	uint8_t pinOpType;
	uint8_t pinAltMode;
	bool isPolling;
} GPIO_PinConfig_t;


#define GPIO_MODE_IN		00
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALT		2
#define GPIO_MODE_AN		3
#define GPIO_MODE_INT_FT	4
#define GPIO_MODE_INT_RT	5
#define GPIO_MODE_INT_RFT	6
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

void GPIO_ClockControl(GPIO_PinConfig_t *config, bool isEna);
void GPIO_Init(GPIO_PinConfig_t *config);
void GPIO_DeInit(GPIO_PinConfig_t *config);
uint8_t GPIO_ReadInputPin(GPIO_PinConfig_t *config);
uint16_t GPIO_ReadInputPort(GPIO_PinConfig_t *config);
void GPIO_WriteOutputPin(GPIO_PinConfig_t *config, uint8_t data);
void GPIO_WriteOutputPort(GPIO_PinConfig_t *config, uint16_t data);
void GPIO_TogglePin(GPIO_PinConfig_t *config);
void GPIO_IRQConfig();
void GPIO_IRQ_Handler();

#endif /* INC_GPIO_H_ */
