#include <button.h>

callback g_EXTI15_10Callback;

void EXTI15_10_IRQHandler(void)
{
	g_EXTI15_10Callback();
}

void ButtonInit(void)
{
	RCC->CR |= 1;
	RCC_GPIO_CLK_EN(PORTC);

	GPIOC->CRH &= 0; // Input mode
	GPIOC->CRH |= (2 << 22); // Enable pull resistor
	GPIOC->ODR |= (1 << 13); // Pull up

	/* Setup interrupt */
	RCC_AFIO_CLK_EN;
	AFIO->EXTICR[3] |= (2 << 4); // Select PC13 as source input
	EXTI->IMR |= (1 << BUTTON);  // Not mask EXTI13
	// Trigger at rising edge
	EXTI->RTSR |= (1 << BUTTON);
	EXTI->FTSR &= (1 << BUTTON);

	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void ButtonInstallCallback(callback func)
{
	g_EXTI15_10Callback = func;
}
