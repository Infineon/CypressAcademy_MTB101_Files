/*******************************************************************************
 * File Name:   lowpower_task.c
 *
 * Description: This file contains the task definition for initializing the
 * Wi-Fi device, configuring the selected WLAN power save mode, connecting to
 * the AP, and suspending the network stack to enable higher power savings on
 * the PSoC 6 MCU.
 *
 * Related Document: See Readme.md
 *
********************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/* Header file includes */

#include "cyhal.h"
#include "cybsp.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>

#include "lowpower_task.h"

/* Wi-Fi Connection Manager (WCM) header file. */
#include "cy_wcm.h"

/* lwIP header files */
#include <cy_lwip.h>
#include "lwip/netif.h"

/* Low Power Assistant header files. */
#include "network_activity_handler.h"

/* Wi-Fi Host Driver (WHD) header files. */
#include "whd_wifi_api.h"


/*******************************************************************************
 * Macros
 ******************************************************************************/
#define MAX_WIFI_RETRY_COUNT    (3)


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
/* Low-power task handle */
TaskHandle_t lowpower_task_handle;


/*******************************************************************************
 * Function definitions
 ******************************************************************************/


/*******************************************************************************
 * Function Name: lowpower_task
 *******************************************************************************
 * Summary:
 * The task initializes the Wi-Fi, LPA (Low-Power Assist middleware) and the OLM
 * (Offload Manager). The Wi-Fi then joins with Access Point with the provided
 * SSID and PASSWORD.After successfully connecting to the network the task
 * suspends the lwIP network stack indefinitely which helps RTOS to enter the
 * Idle state, and then eventually into deep-sleep power mode. The MCU will stay
 * in deep-sleep power mode till the network stack resumes. The network stack
 * resumes whenever any Tx/Rx activity detected in the EMAC interface (path
 * between Wi-Fi driver and network stack).
 *
 * Parameters:
 *  void *arg: Task specific arguments. Never used.
 *
 * Return:
 *  static void: Should never return.
 *
 ******************************************************************************/
void lowpower_task(void *arg)
{
    cy_rslt_t result;
    struct netif *wifi;
    cy_wcm_config_t wcm_config = { .interface = CY_WCM_INTERFACE_TYPE_STA };

    /* Initializes the Wi-Fi device and lwIP stack.*/
    result = cy_wcm_init(&wcm_config);

    if (CY_RSLT_SUCCESS != result)
    {
        ERR_INFO(("Failed to initialize Wi-Fi Connection Manager.\n"));
        CY_ASSERT(0);
    }

    /* Connect to Wi-Fi AP. */
    result = wifi_connect();

    if (CY_RSLT_SUCCESS != result)
    {
        ERR_INFO(("Failed to connect to AP.\n"));
        CY_ASSERT(0);
    }

    /* Obtain the pointer to the lwIP network interface. This pointer is used to
     * access the Wi-Fi driver interface to configure the WLAN power-save mode.
     */
    wifi = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);

    /* Configure the WLAN device to a power-save mode configured by the macro
     * WLAN_POWERSAVE_MODE
     */
    result = wlan_powersave_handler(wifi, WLAN_POWERSAVE_MODE);

    if(CY_RSLT_SUCCESS != result)
    {
        ERR_INFO(("Failed to configure power-save mode for the WLAN device\n"));
        CY_ASSERT(0);
    }

    while (true)
    {
        /* Configures an emac activity callback to the Wi-Fi interface and
         * suspends the network if the network is inactive for a duration of
         * INACTIVE_WINDOW_MS inside an interval of INACTIVE_INTERVAL_MS. The
         * callback is used to signal the presence/absence of network activity
         * to resume/suspend the network stack.
         */
        wait_net_suspend(wifi, portMAX_DELAY, INACTIVE_INTERVAL_MS, INACTIVE_WINDOW_MS);
        cyhal_gpio_toggle(USER_LED);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DELAY_MS));
        cyhal_gpio_toggle(USER_LED);
    }
}


/*******************************************************************************
 * Function Name: wifi_connect
 *******************************************************************************
 * Summary: This function executes a connect to the AP. The maximum number of
 * times it attempts to connect to the AP is specified by MAX_RETRY_COUNT.
 *
 * Parameters:
 * None
 *
 * Return:
 * cy_rslt_t: It contains the status of operation of connecting to the
 * specified AP.
 *
 ******************************************************************************/
cy_rslt_t wifi_connect(void)
{
    cy_rslt_t result;
    cy_wcm_connect_params_t connect_param;
    cy_wcm_ip_address_t ip_address;

    memset(&connect_param, 0, sizeof(cy_wcm_connect_params_t));
    memcpy(&connect_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(&connect_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
    connect_param.ap_credentials.security = WIFI_SECURITY;
    APP_INFO(("Connecting to AP\n"));

    /* Attempt to connect to Wi-Fi until a connection is made or until
     * MAX_WIFI_RETRY_COUNT attempts have been made.
     */
    for(uint32_t conn_retries = 0; conn_retries < MAX_WIFI_RETRY_COUNT; conn_retries++ )
    {
        result = cy_wcm_connect_ap(&connect_param, &ip_address);

        if(result == CY_RSLT_SUCCESS)
        {
            APP_INFO(("Successfully connected to Wi-Fi network '%s'.\n", connect_param.ap_credentials.SSID));

            if (CY_WCM_IP_VER_V4 == ip_address.version)
            {
                APP_INFO(("Assigned IP address = %s\n", ip4addr_ntoa((const ip4_addr_t *)&ip_address.ip.v4)));
            }
            else if(CY_WCM_IP_VER_V6 == ip_address.version)
            {
                APP_INFO(("Assigned IP address = %s\n", ip6addr_ntoa((const ip6_addr_t *)&ip_address.ip.v6)));
            }

            break;
        }

        ERR_INFO(("Connection to Wi-Fi network failed with error code %d."
               "Retrying in %d ms...\n", (int)result, WIFI_CONN_RETRY_INTERVAL_MSEC));
               
        vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MSEC));
    }

    return result;
}


/*******************************************************************************
* Function Name: wlan_powersave_handler
********************************************************************************
*
* Summary: This function configures the power-save mode of the WLAN device.
* There are three power-save modes supported as defined in the enumeration,
* wlan_powersave_mode_t.
*
* Parameters:
* struct netif *wifi: This is the pointer to lwIP's network interface. It 
* contains pointers to the Wi-Fi driver and other network parameters.
* wlan_powersave_mode_t mode: This enumeration contains information about the
* power-save mode which is to be configured for the Wi-Fi interface.
*
* Return:
* cy_rslt_t: It contains the status of operation of configuring the WLAN
* interface's power-save mode.
*
*******************************************************************************/
cy_rslt_t wlan_powersave_handler(struct netif *wifi, enum wlan_powersave_mode_t mode)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    whd_interface_t ifp;
    whd_security_t security_param;
    whd_bss_info_t ap_info;

    if (wifi->flags & NETIF_FLAG_UP)
    {
        /* Get the instance of the WLAN interface.*/
        ifp = (whd_interface_t)wifi->state;

        /* Obtain network parameters configured in the AP.*/
        result = whd_wifi_get_ap_info(ifp, &ap_info, &security_param);

        if (CY_RSLT_SUCCESS == result)
        {
            APP_INFO(("Beacon period = %d, DTIM period = %d\n",
                      ap_info.beacon_period, ap_info.dtim_period));
        }
        else
        {
            ERR_INFO(("Failed to get AP info.\n"));
        }

        /* Configure power-save mode of the WLAN device.*/
        switch (mode)
        {
        case POWERSAVE_WITHOUT_THROUGHPUT:
            result = whd_wifi_enable_powersave(ifp);

            if (CY_RSLT_SUCCESS != result)
            {
                ERR_INFO(("Failed to enable PM1 mode\n"));
            }

            break;
        case POWERSAVE_WITH_THROUGHPUT:
            result = whd_wifi_enable_powersave_with_throughput(ifp, RETURN_TO_SLEEP_MS);

            if (CY_RSLT_SUCCESS != result)
            {
                ERR_INFO(("Failed to enable PM2 mode\n"));
            }

            break;
        case POWERSAVE_DISABLED:
            result = whd_wifi_disable_powersave(ifp);

            if (CY_RSLT_SUCCESS != result)
            {
                ERR_INFO(("Failed to disable powersave\n"));
            }

            break;
        default:
            APP_INFO(("Unknown mode\n"));
            break;
        }
    }
    else
    {
        ERR_INFO(("Wi-Fi interface is not powered on. Failed to configure power-save mode\n"));
        result = CY_RSLT_TYPE_ERROR;
    }

    return result;
}


/* [] END OF FILE */
