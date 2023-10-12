#include <led.h>

int main()
{
	LED_Init();

	while (1)
	{
		LED_Toggle();
		delay();
	}

	return 0;
}
