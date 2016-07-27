/* Board Header files */
#include "../Board.h"

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();
    Board_initI2C();
    Board_initPWM();
    Board_initSPI();

    /* Turn on user LED */
    GPIO_write(Board_LED_GREEN, Board_LED_ON);
    
    /* Start BIOS */
    BIOS_start();

    return (0);
}
