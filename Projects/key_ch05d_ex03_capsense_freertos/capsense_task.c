/*******************************************************************************
* File Name: capsense_task.c
*
* Description: This file contains the task that handles touch sensing.
*
* Related Document: README.md
*
********************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
********************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
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
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*****************************************​**************************************/


/******************************************************************************
* Header files includes
******************************************************************************/
#include "capsense_task.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "led_task.h"


/*******************************************************************************
* Global constants
*******************************************************************************/
#define CAPSENSE_INTERRUPT_PRIORITY    (7u)
#define EZI2C_INTERRUPT_PRIORITY    (6u)    /* EZI2C interrupt priority must be
                                             * higher than CapSense interrupt
                                             */
#define CAPSENSE_SCAN_INTERVAL_MS    (10u)   /* in milliseconds*/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static cy_status capsense_init(void);
static void tuner_init(void);
static void process_touch(void);
static void capsense_isr(void);
static void capsense_end_of_scan_callback(cy_stc_active_scan_sns_t* active_scan_sns_ptr);
static void capsense_timer_callback(TimerHandle_t xTimer);
static void handle_ezi2c_tuner_event(void *callback_arg, cyhal_ezi2c_status_t event);
void handle_error(void);

/******************************************************************************
* Global variables
******************************************************************************/
QueueHandle_t capsense_command_q;
TimerHandle_t scan_timer_handle;
cy_stc_scb_ezi2c_context_t ezi2c_context;
cyhal_ezi2c_t sEzI2C;
cyhal_ezi2c_slave_cfg_t sEzI2C_sub_cfg;
cyhal_ezi2c_cfg_t sEzI2C_cfg;

/* CapSense Deep Sleep handling setup */
/* Parameter structure for Sleep callback function */
cy_stc_syspm_callback_params_t CapSenseDeepSleepParams =
{
    CYBSP_CSD_HW,           // CapSense HW block base address
    &cy_capsense_context    // CapSense context
};

/* Callback Structure for Sleep callback function*/
cy_stc_syspm_callback_t CapSenseDeepSleep =
{
    &Cy_CapSense_DeepSleepCallback,  //Provided by CapSense API
    CY_SYSPM_DEEPSLEEP,  // Callback for Deep Sleep Events
    0U,    // Callback is executed for all DeepSleep events
    &CapSenseDeepSleepParams,  // Callback parameters structure
    NULL,  // No previous callback in the list
    NULL,  // No next callback in the list
    0U     // Priority order
};


/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}


/*******************************************************************************
* Function Name: task_capsense
********************************************************************************
* Summary:
*  Task that initializes the CapSense block and processes the touch input.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void task_capsense(void* param)
{
    BaseType_t rtos_api_result;
    cy_status status;
    capsense_command_t capsense_cmd;

    /* Remove warning for unused parameter */
    (void)param;

    /* Initialize timer for periodic CapSense scan */
    scan_timer_handle = xTimerCreate ("Scan Timer", CAPSENSE_SCAN_INTERVAL_MS,
                                      pdTRUE, NULL, capsense_timer_callback);

    /* Setup communication between Tuner GUI and PSoC 6 MCU */
    tuner_init();

    /* Initialize CapSense block */
    status = capsense_init();
    if(CY_RET_SUCCESS != status)
    {
        CY_ASSERT(0u);
    }

    /* Register CapSense Deep Sleep event callback */
    Cy_SysPm_RegisterCallback(&CapSenseDeepSleep);

    /* Start the timer */
    xTimerStart(scan_timer_handle, 0u);

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a CapSense command has been received over queue */
        rtos_api_result = xQueueReceive(capsense_command_q, &capsense_cmd,
                                        portMAX_DELAY);

        /* Command has been received from capsense_cmd */
        if(rtos_api_result == pdTRUE)
        {
            /* Check if CapSense is busy with a previous scan */
            if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
            {
                switch(capsense_cmd)
                {
                    case CAPSENSE_SCAN:
                    { 
                        /* Start scan */
                        Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
                        break;
                    }
                    case CAPSENSE_PROCESS:
                    {
                        /* Process all widgets */
                        Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
                        process_touch();

                        /* Establishes synchronized operation between the CapSense
                         * middleware and the CapSense Tuner tool.
                         */
                        Cy_CapSense_RunTuner(&cy_capsense_context);
                        break;
                    }
                    /* Invalid command */
                    default:
                    {
                        break;
                    }
                }
            }
        }
        /* Task has timed out and received no data during an interval of
         * portMAXDELAY ticks.
         */
        else
        {
            /* Handle timeout here */
        }
    }
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  This function processes the touch input and sends command to LED task.
*
*******************************************************************************/
static void process_touch(void)
{
    /* Variables used to store touch information */
    uint32_t button0_status = 0;
    uint32_t button1_status = 0;
    uint16_t slider_pos = 0;
    uint8_t slider_touched = 0;
    cy_stc_capsense_touch_t *slider_touch;

    /* Variables used to store previous touch information */
    static uint32_t button0_status_prev = 0;
    static uint32_t button1_status_prev = 0;
    static uint16_t slider_pos_perv = 0;

    /* Variables used for storing command and data for LED Task */
    led_command_data_t led_cmd_data;
    bool send_led_command = false;

    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON0_WDGT_ID,
        CY_CAPSENSE_BUTTON0_SNS0_ID,
        &cy_capsense_context);

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON1_WDGT_ID,
        CY_CAPSENSE_BUTTON1_SNS0_ID,
        &cy_capsense_context);

    /* Get slider status */
    slider_touch = Cy_CapSense_GetTouchInfo(
        CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
        &cy_capsense_context);
    slider_pos = slider_touch->ptrPosition->x;
    slider_touched = slider_touch->numPosition;

    /* Detect new touch on Button0 */
    if((0u != button0_status) && (0u == button0_status_prev))
    {
        led_cmd_data.command = LED_TURN_ON;
        send_led_command = true;
    }

    /* Detect new touch on Button1 */
    if((0u != button1_status) && (0u == button1_status_prev))
    {
        led_cmd_data.command = LED_TURN_OFF;
        send_led_command = true;
    }

    /* Detect new touch on slider */
    if((0u != slider_touched) && (slider_pos_perv != slider_pos ))
    {
        led_cmd_data.command = LED_UPDATE_BRIGHTNESS;
      
        /* brightness in percentage */
        led_cmd_data.brightness = (slider_pos * 100) 
                    / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;
        send_led_command = true;
    }

    /* Send command to update LED state if required */
    if(send_led_command)
    {
        xQueueSendToBack(led_command_data_q, &led_cmd_data, 0u);
    }

    /* Update previous touch status */
    button0_status_prev = button0_status;
    button1_status_prev = button1_status;
    slider_pos_perv = slider_pos;
}


/*******************************************************************************
* Function Name: capsense_init
********************************************************************************
* Summary:
*  This function initializes the CSD HW block, and configures the CapSense
*  interrupt.
*
*******************************************************************************/
static cy_status capsense_init(void)
{
    cy_status status = CYRET_SUCCESS;

    /* CapSense interrupt configuration parameters */
    static const cy_stc_sysint_t capSense_intr_config =
    {
        .intrSrc = csd_interrupt_IRQn,
        .intrPriority = CAPSENSE_INTERRUPT_PRIORITY,
    };

    /*Initialize CapSense Data structures */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    Cy_SysInt_Init(&capSense_intr_config, &capsense_isr);
    NVIC_ClearPendingIRQ(capSense_intr_config.intrSrc);
    NVIC_EnableIRQ(capSense_intr_config.intrSrc);

    /* Register end of scan callback */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
                                              capsense_end_of_scan_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }
    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }
    
    return status;
}


/*******************************************************************************
* Function Name: capsense_end_of_scan_callback
********************************************************************************
* Summary:
*  CapSense end of scan callback function. This function sends a command to
*  CapSense task to process scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t * active_scan_sns_ptr (unused)
*
*******************************************************************************/
static void capsense_end_of_scan_callback(cy_stc_active_scan_sns_t* active_scan_sns_ptr)
{
    BaseType_t xYieldRequired;

    (void)active_scan_sns_ptr;

    /* Send command to process CapSense data */
    capsense_command_t commmand = CAPSENSE_PROCESS;
    xYieldRequired = xQueueSendToBackFromISR(capsense_command_q, &commmand, 0u);
    portYIELD_FROM_ISR(xYieldRequired);
}


/*******************************************************************************
* Function Name: capsense_timer_callback
********************************************************************************
* Summary:
*  CapSense timer callback. This function sends a command to start CapSense
*  scan.
*
* Parameters:
*  TimerHandle_t xTimer (unused)
*
*******************************************************************************/
static void capsense_timer_callback(TimerHandle_t xTimer)
{
    capsense_command_t command = CAPSENSE_SCAN;
    BaseType_t xYieldRequired;

    (void)xTimer;

    /* Send command to start CapSense scan */
    xYieldRequired = xQueueSendToBackFromISR(capsense_command_q, &command, 0u);
    portYIELD_FROM_ISR(xYieldRequired);
}


/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CSD block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}


/*******************************************************************************
* Function Name: ezi2c_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from EZI2C block.
*
*******************************************************************************/
static void handle_ezi2c_tuner_event(void *callback_arg, cyhal_ezi2c_status_t event)
{
    cyhal_ezi2c_status_t status;
    cy_stc_scb_ezi2c_context_t *context = &ezi2c_context;

    /* Get the slave interrupt sources */
    status = cyhal_ezi2c_get_activity_status(&sEzI2C);

    /* Handle the error conditions */
    if (0UL != (CYHAL_EZI2C_STATUS_ERR & status))
    {
        handle_error();
    }

    /* Handle the receive direction (master writes data) */
    if (0 != (CYHAL_EZI2C_STATUS_READ1 & status))
    {
        cyhal_i2c_slave_config_write_buffer((cyhal_i2c_t *)&sEzI2C, context->curBuf, context->bufSize);
    }
    /* Handle the transmit direction (master reads data) */
    if (0 != (CYHAL_EZI2C_STATUS_WRITE1 & status))
    {
        cyhal_i2c_slave_config_read_buffer((cyhal_i2c_t *)&sEzI2C, context->curBuf, context->bufSize);
    }
}


/*******************************************************************************
* Function Name: tuner_init
********************************************************************************
* Summary:
*  Initializes communication between Tuner GUI and PSoC 6 MCU.
*
*******************************************************************************/
static void tuner_init(void)
{
    cy_rslt_t result;
    /* Configure Capsense Tuner as EzI2C Slave */
    sEzI2C_sub_cfg.buf = (uint8 *)&cy_capsense_tuner;
    sEzI2C_sub_cfg.buf_rw_boundary = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.buf_size = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.slave_address = 8U;

    sEzI2C_cfg.data_rate = CYHAL_EZI2C_DATA_RATE_400KHZ;
    sEzI2C_cfg.enable_wake_from_sleep = false;
    sEzI2C_cfg.slave1_cfg = sEzI2C_sub_cfg;
    sEzI2C_cfg.sub_address_size = CYHAL_EZI2C_SUB_ADDR16_BITS;
    sEzI2C_cfg.two_addresses = false;
    result = cyhal_ezi2c_init( &sEzI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL, &sEzI2C_cfg);
    if (result != CY_RSLT_SUCCESS)
        {
            handle_error();
        }
    cyhal_ezi2c_register_callback( &sEzI2C, handle_ezi2c_tuner_event, NULL);
    cyhal_ezi2c_enable_event(&sEzI2C,
                             (CYHAL_EZI2C_STATUS_ERR | CYHAL_EZI2C_STATUS_WRITE1 | CYHAL_EZI2C_STATUS_READ1),
                             EZI2C_INTERRUPT_PRIORITY, true);

}

/* END OF FILE [] */
