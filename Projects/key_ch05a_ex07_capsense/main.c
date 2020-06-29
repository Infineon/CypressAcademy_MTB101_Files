/******************************************************************************
* File Name: main.c
*
* Description: This code example features a 5-segment CapSense slider and two
*              CapSense buttons. Button 0 turns the LED ON, Button 1 turns the
*              LED OFF and the slider controls the brightness of the LED. The
*              code example also features interfacing with Tuner GUI using I2C
*              interface.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation. All rights reserved.
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


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "led.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define CSD_COMM_HW             (SCB3)
#define CSD_COMM_IRQ            (scb_3_interrupt_IRQn)
#define CSD_COMM_PCLK           (PCLK_SCB3_CLOCK)
#define CSD_COMM_CLK_DIV_HW     (CY_SYSCLK_DIV_8_BIT)
#define CSD_COMM_CLK_DIV_NUM    (1U)
#define CSD_COMM_CLK_DIV_VAL    (3U)
#define CSD_COMM_SCL_PORT       (GPIO_PRT6)
#define CSD_COMM_SCL_PIN        (0u)
#define CSD_COMM_SDA_PORT       (GPIO_PRT6)
#define CSD_COMM_SDA_PIN        (1u)
#define CSD_COMM_SCL_HSIOM_SEL  (P6_0_SCB3_I2C_SCL)
#define CSD_COMM_SDA_HSIOM_SEL  (P6_1_SCB3_I2C_SDA)
#define CAPSENSE_INTR_PRIORITY  (7u)
#define EZI2C_INTR_PRIORITY     (6u)    /* EZI2C interrupt priority must be
                                         * higher than CapSense interrupt.
                                         * Lower number mean higher priority.
                                         */


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static cy_status initialize_capsense(void);
static void process_touch(void);
static void initialize_capsense_tuner(void);
static void capsense_isr(void);


/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize CapSense
*  - initialize tuner communication
*  - scan touch input continuously and update the LED accordingly.
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_status status;
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

    initialize_led();
    initialize_capsense_tuner();
    status = initialize_capsense();
    
    if(CYRET_SUCCESS != status)
    {
        /* Halt the CPU if CapSense initialization failed */
        CY_ASSERT(0);
    }

    /* Start first scan */
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for(;;)
    {
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Process touch input */
            process_touch();

            /* Establishes synchronized operation between the CapSense
             * middleware and the CapSense Tuner tool.
             */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /* Start next scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        }
    }
}


/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
static void process_touch(void)
{
    uint32_t button0_status;
    uint32_t button1_status;
    cy_stc_capsense_touch_t* slider_touch_info;
    uint16_t slider_pos;
    uint8_t slider_touch_status ;
    bool led_update_req = false;

    static uint32_t button0_status_prev;
    static uint32_t button1_status_prev;
    static uint16_t slider_pos_prev;
    static led_data_t led_data = {LED_ON, LED_MAX_BRIGHTNESS};


    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(
                                CY_CAPSENSE_BUTTON0_WDGT_ID,
                                CY_CAPSENSE_BUTTON0_SNS0_ID,
                                &cy_capsense_context);

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(
                                CY_CAPSENSE_BUTTON1_WDGT_ID,
                                CY_CAPSENSE_BUTTON0_SNS0_ID,
                                &cy_capsense_context);

    /* Get slider status */
    slider_touch_info = Cy_CapSense_GetTouchInfo(
            CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    slider_touch_status = slider_touch_info->numPosition;
    slider_pos = slider_touch_info->ptrPosition->x;


    /* Detect new touch on Button0 */
    if((0u != button0_status) &&
       (0u == button0_status_prev))
    {
        led_data.state = LED_ON;
        led_update_req = true;
    }

    /* Detect new touch on Button1 */
    if((0u != button1_status) &&
       (0u == button1_status_prev))
    {
        led_data.state = LED_OFF;
        led_update_req = true;
    }

    /* Detect the new touch on slider */
    if((0 != slider_touch_status) &&
       (slider_pos != slider_pos_prev))
    {
        led_data.brightness = (slider_pos * 100) / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;
        led_update_req = true;
    }

    /* Update the LED state if requested */
    if(led_update_req)
    {
        update_led_state(&led_data);
    }

    /* Update previous touch status */
    button0_status_prev = button0_status;
    button1_status_prev = button1_status;
    slider_pos_prev = slider_pos;
}


/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
*
*******************************************************************************/
static cy_status initialize_capsense(void)
{
    cy_status status;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
    {
        .intrSrc = CYBSP_CSD_IRQ,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if(CYRET_SUCCESS == status)
    {
        /* Initialize CapSense interrupt */
        Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
        NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
        NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }
    
    return status;
}


/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
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
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CSD_COMM_HW, &ezi2c_context);
}


/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
*  Initializes interface between Tuner GUI and PSoC 6 MCU.
*
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    /* EZI2C configuration structure */
    const cy_stc_scb_ezi2c_config_t csd_comm_config =
    {
        .numberOfAddresses = CY_SCB_EZI2C_ONE_ADDRESS,
        .slaveAddress1 = 8U,
        .slaveAddress2 = 0U,
        .subAddressSize = CY_SCB_EZI2C_SUB_ADDR16_BITS,
        .enableWakeFromSleep = false,
    };

    /* EZI2C interrupt configuration structure */
    static const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = CSD_COMM_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize EZI2C pins */
    Cy_GPIO_Pin_FastInit(CSD_COMM_SCL_PORT, CSD_COMM_SCL_PIN,
                         CY_GPIO_DM_OD_DRIVESLOW, 1, CSD_COMM_SCL_HSIOM_SEL);
    Cy_GPIO_Pin_FastInit(CSD_COMM_SDA_PORT, CSD_COMM_SDA_PIN,
                         CY_GPIO_DM_OD_DRIVESLOW, 1, CSD_COMM_SDA_HSIOM_SEL);

    /* Configure EZI2C clock */
    Cy_SysClk_PeriphDisableDivider(CSD_COMM_CLK_DIV_HW, CSD_COMM_CLK_DIV_NUM);
    Cy_SysClk_PeriphAssignDivider(CSD_COMM_PCLK, CSD_COMM_CLK_DIV_HW,
                                  CSD_COMM_CLK_DIV_NUM);
    Cy_SysClk_PeriphSetDivider(CSD_COMM_CLK_DIV_HW, CSD_COMM_CLK_DIV_NUM,
                               CSD_COMM_CLK_DIV_VAL);
    Cy_SysClk_PeriphEnableDivider(CSD_COMM_CLK_DIV_HW, CSD_COMM_CLK_DIV_NUM);
    

    /* Initialize EZI2C */
    Cy_SCB_EZI2C_Init(CSD_COMM_HW, &csd_comm_config, &ezi2c_context);

    /* Initialize and enable EZI2C interrupts */
    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set up communication data buffer to CapSense data structure to be exposed
     * to I2C master at primary slave address request.
     */
    Cy_SCB_EZI2C_SetBuffer1(CSD_COMM_HW, (uint8 *)&cy_capsense_tuner,
                            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
                            &ezi2c_context);

    /* Enable EZI2C block */
    Cy_SCB_EZI2C_Enable(CSD_COMM_HW);
}


/* [] END OF FILE */
