/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 6 MCU Hello World Example
*              for ModusToolbox.
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
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
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/
// Comment out this line to use CPU Sleep instead of Deep Sleep
#define USE_DEEP_SLEEP

/* LED blink timer clock value in Hz  */
#define LED_BLINK_TIMER_CLOCK_HZ          (10000)

/* LED blink timer period value */
#define LED_BLINK_TIMER_PERIOD            (9999)


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_init(void);

#ifdef USE_DEEP_SLEEP
static void isr_timer(void *callback_arg, cyhal_lptimer_event_t event);
#else
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);
#endif

/*******************************************************************************
* Global Variables
*******************************************************************************/
bool timer_interrupt_flag = false;
bool led_blink_active_flag = true;

#ifndef USE_DEEP_SLEEP
/* UART HAL object used by Retarget-IO for Debug UART port */
extern cyhal_uart_t cy_retarget_io_uart_obj;

/* Variable for storing character read from terminal */
uint8_t uart_read_value;

bool uart_command_flag = false;
#endif

/* Timer object used for blinking the LED */
#ifdef USE_DEEP_SLEEP
cyhal_lptimer_t led_blink_timer;
#else
cyhal_timer_t led_blink_timer;
#endif


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It sets up a timer to trigger a 
* periodic interrupt. The main while loop checks for the status of a flag set 
* by the interrupt and toggles an LED at 1Hz to create an LED blinky. The 
* while loop also checks whether the 'Enter' key was pressed and 
* stops/restarts LED blinking.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();


#ifndef USE_DEEP_SLEEP
    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
#endif

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, 
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

#ifndef USE_DEEP_SLEEP
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "PSoC 6 MCU: Hello World! Example "
           "****************** \r\n\n");

    printf("Hello World!!!\r\n\n");
    
    printf("For more PSoC 6 MCU projects, "
           "visit our code examples repositories:\r\n\n");

    printf("1. ModusToolbox Examples:\r\n https://www.cypress.com/documentation"
            "/code-examples/psoc6-sdk-code-examples-modustoolbox-software\r\n\n");

    printf("2. Mbed OS Examples:\r\n https://www.cypress.com/documentation/"
           "code-examples/mbed-sdk-code-examples\r\n\n");
#endif

    /* Initialize timer to toggle the LED */
    timer_init();

#ifndef USE_DEEP_SLEEP

    printf("Press 'Enter' key to pause or "
           "resume blinking the user LED \r\n\r\n");
#endif

    for(;;)
    {
#ifdef USE_DEEP_SLEEP
    	cyhal_syspm_deepsleep();
#else
    	cyhal_syspm_sleep();
#endif

#ifndef USE_DEEP_SLEEP
    	if(cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1) \
    			== CY_RSLT_SUCCESS)
		{
			if (uart_read_value == '\r')
			{
				led_blink_active_flag ^= 1;
				uart_command_flag = true;
			}
		}
#endif

		if(timer_interrupt_flag)
		{
			timer_interrupt_flag = false;
#ifndef USE_DEEP_SLEEP
			if (uart_command_flag)
			{
				uart_command_flag = false;
				if (led_blink_active_flag)
				{
					printf("LED blinking resumed\r\n");
				}
				else
				{
					printf("LED blinking paused \r\n");
				}
				printf("\x1b[1F");
			}
#endif
			if (led_blink_active_flag)
			{
				cyhal_gpio_toggle((cyhal_gpio_t) CYBSP_USER_LED);
			}
		}
	}
}


/*******************************************************************************
* Function Name: timer_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks 
* continuously and produces a periodic interrupt on every terminal count 
* event. The period is defined by the 'period' and 'compare_value' of the 
* timer configuration structure 'led_blink_timer_cfg'. Without any changes, 
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
*******************************************************************************/
#ifdef USE_DEEP_SLEEP

#define LPTIMER_MATCH_VALUE (32767)
#define LPTIMER_INTR_PRIORITY (3u)

void timer_init(void)
{
   /* Initialize lptimer. */
   cyhal_lptimer_init(&led_blink_timer );

   /* CLK_LF is 32,768 Hz, so 32,767 counts give us a 1 second interrupt */
   cyhal_lptimer_set_match(&led_blink_timer, LPTIMER_MATCH_VALUE);

   /* Register the interrupt callback handler */
   cyhal_lptimer_register_callback(&led_blink_timer, isr_timer, NULL);

   /* Configure and Enable the LPTIMER events */
   cyhal_lptimer_enable_event(&led_blink_timer,
             CYHAL_LPTIMER_COMPARE_MATCH, LPTIMER_INTR_PRIORITY, true);

   /* Reload/Reset the Low-Power timer to get periodic interrupt. */
   cyhal_lptimer_reload(&led_blink_timer);
}

static void isr_timer(void *callback_arg, cyhal_lptimer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /* Set the interrupt flag and process it from the main loop */
    timer_interrupt_flag = true;

    /* Reload/Reset the LPTIMER to get periodic interrupt */
    cyhal_lptimer_reload(&led_blink_timer);
}

#else

void timer_init(void)
 {
    cy_rslt_t result;

    const cyhal_timer_cfg_t led_blink_timer_cfg = 
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = LED_BLINK_TIMER_PERIOD,   /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&led_blink_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction, 
       duration */
    cyhal_timer_configure(&led_blink_timer, &led_blink_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&led_blink_timer, isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                              7, true);

    /* Start the timer with the configured settings */
    cyhal_timer_start(&led_blink_timer);
 }


/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    timer_interrupt_flag = true;
}

#endif
/* [] END OF FILE */
