#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "cy_retarget_io.h"
#include <stdio.h>

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    cyhal_gpio_init(CYBSP_USER_LED4,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init(CYBSP_USER_LED3,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);

    printf("PSoC Rocks!\r\n");

    for(;;)
    {
        cyhal_gpio_toggle(CYBSP_USER_LED4);
        cyhal_gpio_toggle(CYBSP_USER_LED3);

        Cy_SysLib_Delay(500);
    }
}
