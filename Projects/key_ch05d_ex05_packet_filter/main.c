/*******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for WLAN Lowpower Example in
 * ModusToolbox.
 *
 * Related Document: See Readme.md
 *
 *******************************************************************************
 * (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 ******************************************************************************/

/* Header file includes */

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>

/*Task header file.*/
#include "lowpower_task.h"


/*******************************************************************************
 * Macros
 ******************************************************************************/
#define LOW_POWER_TASK_STACK_SIZE_BYTES   (1024u)
#define LOW_POWER_TASK_PRIORITY           (3u)


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority;


/*******************************************************************************
 * Function definitions
 ******************************************************************************/


/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 *  System entrance point. This function sets up user tasks and then starts
 *  the RTOS scheduler.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main()
{
    cy_rslt_t result;

    /* This enables RTOS aware debugging in OpenOCD. */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the board support package. */
    result = cybsp_init();
    CHECK_RESULT(result);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize LED. */
    result = cyhal_gpio_init(USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    CHECK_RESULT(result);

    /* Initialize retarget-io to use the debug UART port. */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CHECK_RESULT(result);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    printf("\x1b[2J\x1b[;H");
    printf("===============================================================\n");
    printf("CE230106 - AnyCloud Example: WLAN Lowpower\n");
    printf("===============================================================\n\n");

    /* Create a task that initializes the Wi-Fi device, configures it
     * in the specified WLAN power save mode and suspends the network stack
     * indefinitely until there is network activity detected by the WLAN device.
     */
    xTaskCreate(lowpower_task, "Low power task", LOW_POWER_TASK_STACK_SIZE_BYTES, NULL, LOW_POWER_TASK_PRIORITY, &lowpower_task_handle);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);
}


/* [] END OF FILE */
