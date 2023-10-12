/*
 * gpio.c
 *
 *  Created on: Oct 6, 2023
 *      Author: ADMIN
 */

#include <gpio.h>

void GPIO_ClockControl(GPIO_PinConfig_t *config, bool isEna)
{
	if (isEna)
	{
		RCC_GPIO_CLK_EN(config->port);
	} else {
		RCC_GPIO_CLK_DI(config->port);
	}
}

void GPIO_Init(GPIO_PinConfig_t *config)
{

}
