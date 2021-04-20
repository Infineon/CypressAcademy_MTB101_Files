/******************************************************************************
* File Name: lowpower_task.h
*
* Description: This file includes the macros and enumerations used by the
* example to connect to an AP, configure power-save mode of the WLAN device, and
* configure the parameters for suspending the network stack.
*
* Related Document: See README.md
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

/*******************************************************************************
* Include guard
*******************************************************************************/
#ifndef LOWPOWER_TASK_H_
#define LOWPOWER_TASK_H_

#include "cy_wcm.h"
#include "cybsp.h"
#include "cy_lwip.h"


/* Wi-Fi Credentials: Modify WIFI_SSID and WIFI_PASSWORD to match your Wi-Fi network
 * Credentials.
 */
#define WIFI_SSID                         "WIFI_SSID"
#define WIFI_PASSWORD                     "WIFI_PASSWORD"

/* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
 * in "cy_wcm.h" for more details.
 */
#define WIFI_SECURITY                     CY_WCM_SECURITY_WPA2_AES_PSK

#define LED_BLINK_DELAY_MS                (100u)

/* This macro specifies the interval in milliseconds that the device monitors
 * the network for inactivity. If the network is inactive for duration lesser 
 * than INACTIVE_WINDOW_MS in this interval, the MCU does not suspend the network 
 * stack and informs the calling function that the MCU wait period timed out 
 * while waiting for network to become inactive.
 */
#define INACTIVE_INTERVAL_MS              300u

/* This macro specifies the continuous duration in milliseconds for which the 
 * network has to be inactive. If the network is inactive for this duaration,
 * the MCU will suspend the network stack. Now, the MCU will not need to service
 * the network timers which allows it to stay longer in sleep/deepsleep.
 */
#define INACTIVE_WINDOW_MS                200u

/* This macro defines the power-save mode that the WLAN device has to be
 * configured to. The valid values for this macro are POWERSAVE_WITH_THROUGHPUT,
 * POWERSAVE_WITHOUT_THROUGHPUT, POWERSAVE_DISABLED
 * which are defined in the enumeration wlan_powersave_mode_t.
 */
#define WLAN_POWERSAVE_MODE               POWERSAVE_WITH_THROUGHPUT

/* This macro specifies the duration in milliseconds for which the STA stays
 * awake after receiving frames from AP in PM2 mode. The delay value must be set
 * to a multiple of 10 and not equal to zero. Minimum value is 10u. Maximum
 * value is 2000u.
 */
#define RETURN_TO_SLEEP_MS                (10u)

/* Delay between successive Wi-Fi connection attempts, in milliseconds. */
#define WIFI_CONN_RETRY_INTERVAL_MSEC     (100u)

/* Select an LED that is not connected to P6_VDD (MCU supply) on the board so
 * that the current consumption measurement for MCU is not affected by the LED
 * current. 
 */
#if defined(TARGET_CY8CPROTO_062_4343W) || defined(TARGET_CY8CKIT_062_WIFI_BT) || defined(TARGET_CYSBSYSKIT_DEV_01)
#define USER_LED                          CYBSP_USER_LED
#elif (defined(TARGET_CY8CKIT_062S2_43012) || defined(TARGET_CYW9P62S1_43438EVB_01) || defined(TARGET_CYW9P62S1_43012EVB_01))
#define USER_LED                          CYBSP_USER_LED2
#endif

#define APP_INFO( x )           do { printf("Info: "); printf x;} while(0);
#define ERR_INFO( x )           do { printf("Error: "); printf x;} while(0);
#define CHECK_RESULT( x )       do { if(CY_RSLT_SUCCESS != x) { CY_ASSERT(0); } } while(0);

/* This data type enlists enumerations that correspond to the different 
 * power-save modes available. They are,
 * POWERSAVE_WITHOUT_THROUGHPUT:This mode corresponds to (legacy) 802.11 PS-Poll
 * mode and should be used to achieve the lowest power consumption possible when
 * the Wi-Fi device is primarily passively listening to the network.
 *
 * POWERSAVE_WITH_THROUGHPUT:This mode attempts to increase throughput by
 * waiting for a timeout period before returning to sleep rather than returning
 * to sleep immediately.
 *
 * POWERSAVE_DISABLED: Powersave mode is disabled.
 */
enum wlan_powersave_mode_t
{
   POWERSAVE_WITHOUT_THROUGHPUT,
   POWERSAVE_WITH_THROUGHPUT,
   POWERSAVE_DISABLED
};


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern TaskHandle_t lowpower_task_handle;


/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void lowpower_task(void *arg);
cy_rslt_t wifi_connect(void);
cy_rslt_t wlan_powersave_handler(struct netif *wifi, enum wlan_powersave_mode_t mode);


#endif /* LOWPOWER_TASK_H_ */


/* [] END OF FILE */
