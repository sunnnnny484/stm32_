#include <stm32f103.h>
#include <led.h>
#include <button.h>

static void button_callback(void)
{
	if (EXTI->PR & (1 << BUTTON))
	{
		EXTI->PR |= (1 << BUTTON);
		LED_Toggle();
	}
}

int main()
{
	LED_Init();

	ButtonInstallCallback(button_callback);
	ButtonInit();

	while (1);

	return 0;
}
