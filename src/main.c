#include "main.h"
#include <stm32f0xx_hal.h>

int main(void)
{
    #if defined (ledStripTest)
        ledStripTestMain();
    #endif
}