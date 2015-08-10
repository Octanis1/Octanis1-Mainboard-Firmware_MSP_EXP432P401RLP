/* Board Header files */
#include "../Board.h"





//called periodically
Void clockFxn(UArg arg)
{
  //flash led
  GPIO_toggle(Board_LED0);

  //print load periodically
  int cpuLoad = Load_getCPULoad();
  cli_printf("CPU load: %d", cpuLoad);

}


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

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);


    /* Start BIOS */
    BIOS_start();

    return (0);
}



