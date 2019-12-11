/******************************************************************************
* File Name: cy_wifi_scan.c
*
* Description: This is the source code Wi-Fi Scanner Project on PSoC 6
*
*******************************************************************************
* (c) 2019, Cypress Semiconductor Corporation. All rights reserved.
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

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "aws_demo_config.h"
#include "task.h"
#include "iot_wifi.h"
#include <stdbool.h>
#include "platform/iot_network.h"


#ifdef CY_USE_LWIP
#include "tcpip.h"
#endif

/* AWS library includes. */
#include "iot_system_init.h"
#include "iot_logging_task.h"
#include "iot_wifi.h"

/* BSP & Abstraction inclues */
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyabs_rtos.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* The task delay for allowing the lower priority logging task to print out Wi-Fi
 * failure status before blocking indefinitely. */
#define mainLOGGING_WIFI_STATUS_DELAY       pdMS_TO_TICKS( 5000 )

/* Maximum number of Wi-Fi networks to scan */
#define MAX_SCAN_NUMBER         100

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Names used for printing the security type of a network */
char *security_string[] =
{
        "None   \0",
        "WEP    \0",
        "WPA    \0",
        "WPA2   \0",
        "Unknown\0"
};

/* An array that stores the results of the Wi-Fi Scan */
WIFIScanResult_t ScanResults[ MAX_SCAN_NUMBER ];


/*******************************************************************************
* Function Name: RunWifiScanDemo
********************************************************************************
* Summary:
*  Scans and prints the Wi-Fi networks. This function runs as a FreeRTOS task.
*
* Parameters:
*  None
*
* Return:
*  int
*******************************************************************************/
int RunWifiScanDemo( void )
{    
    WIFIReturnCode_t WiFiStatus;
    printf("\r\nCypress Wifi Scan\r\n");

    while(1)
    {
        memset( ScanResults, 0x00, sizeof(ScanResults));
        WiFiStatus = WIFI_Scan( ScanResults, MAX_SCAN_NUMBER );
        if ( WiFiStatus == eWiFiSuccess)
        {
            int i;
            printf("\r\n-------------------------------------------------------------------------------");
            printf("\r\nindex   CH   RSSI   Security          BSSID         SSID");
            printf("\r\n-------------------------------------------------------------------------------\r\n");
            for (i = 0; i < MAX_SCAN_NUMBER; i++)
            {
                if ((ScanResults[i].ucBSSID[0] == 0) && (ScanResults[i].ucBSSID[1] == 0) && (ScanResults[i].ucBSSID[2] == 0) &&
                    (ScanResults[i].ucBSSID[3] == 0) && (ScanResults[i].ucBSSID[4] == 0) && (ScanResults[i].ucBSSID[5] == 0) )
                {
                    break;
                }
                printf("[%3i]  %3d   %3d    (%d)%s  %02x.%02x.%02x.%02x.%02x.%02x  %s\r\n",
                        i, ScanResults[i].cChannel, ScanResults[i].cRSSI, ScanResults[i].xSecurity, security_string[ScanResults[i].xSecurity],
                        ScanResults[i].ucBSSID[0], ScanResults[i].ucBSSID[1], ScanResults[i].ucBSSID[2],
                        ScanResults[i].ucBSSID[3], ScanResults[i].ucBSSID[4], ScanResults[i].ucBSSID[5],
                        ScanResults[i].cSSID);
            }
        }
        vTaskDelay( mainLOGGING_WIFI_STATUS_DELAY  );
    }

    return 0;
}


/* [] END OF FILE */

