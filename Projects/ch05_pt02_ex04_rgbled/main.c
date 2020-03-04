#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "cy_rgb_led.h"

#include "cy_retarget_io.h"
#include <stdio.h>

#define DEFINE_TO_STRING(macro) (#macro)

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = cy_rgb_led_init(CYBSP_LED_RGB_RED, CYBSP_LED_RGB_GREEN, CYBSP_LED_RGB_BLUE, CY_RGB_LED_ACTIVE_LOW);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    cyhal_gpio_init(CYBSP_USER_LED4,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init(CYBSP_USER_LED3,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);

    printf("PSoC Rocks!\r\n");

    for(;;)
    {
        printf( "Color is %s\r\n", DEFINE_TO_STRING(CY_RGB_LED_COLOR_YELLOW));
        cy_rgb_led_on(CY_RGB_LED_COLOR_YELLOW, CY_RGB_LED_MAX_BRIGHTNESS);
        Cy_SysLib_Delay( 1000 );

        printf( "Color is %s\r\n", DEFINE_TO_STRING(CY_RGB_LED_COLOR_CYAN));
        cy_rgb_led_on(CY_RGB_LED_COLOR_CYAN, CY_RGB_LED_MAX_BRIGHTNESS);
        Cy_SysLib_Delay( 1000 );

        printf( "Color is %s\r\n", DEFINE_TO_STRING(CY_RGB_LED_COLOR_PURPLE));
        cy_rgb_led_on(CY_RGB_LED_COLOR_PURPLE, CY_RGB_LED_MAX_BRIGHTNESS);
        Cy_SysLib_Delay( 1000 );
    }
}
