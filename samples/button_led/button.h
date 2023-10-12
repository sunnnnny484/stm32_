#ifndef BUTTON_H
#define BUTTON_H

#include <stm32f103.h>

#define BUTTON		13		/* PC13 */

typedef void (*callback)(void);

void ButtonInit(void);
void ButtonInstallCallback(callback func);

#endif
