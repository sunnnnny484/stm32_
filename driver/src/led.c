#include <led.h>

#define LED_GREEN	5	/* PA5 */

void LED_Init(void)
{
	RCC->CR |= 1;
	RCC_GPIO_CLK_EN(PORTA);

	GPIOA->CRL &= 0;
	GPIOA->CRL |= (1 << 20);
//	GPIOA->CRL |= (1 << 22);
}

void LED_DeInit(void)
{
	RCC_GPIO_CLK_DI(PORTA);
}

void LED_On(void)
{
	GPIOA->ODR |= (1 << 5);
}

void LED_Off(void)
{
	GPIOA->ODR &= ~(1 << 5);
}

void LED_Toggle(void)
{
	GPIOA->ODR ^= (1 << 5);
}

void delay(void)
{
	int32_t i = 1000000;
	while (i--);
}
