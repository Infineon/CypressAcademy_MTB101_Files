/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the WiFi Onboarding Using BLE
*              project for ModusToolbox.
*
* Related Document: See Readme.md
*
*******************************************************************************/
/*******************************************************************************
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
*******************************************************************************/

#include <app_utils.h>
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "app_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "cycfg_gatt_db.h"
#include "wifi_task.h"
#include "cyhal.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/* Priority for the button interrupt */
#define GPIO_INTERRUPT_PRIORITY             (7)

/* Number of Advertisement elements */
#define ADV_ELEMENTS 3

/* LE Key Size */
#define MAX_KEY_SIZE (0x10)

/******************************************************************************
 *                             Global Variables
 ******************************************************************************/
/* Task Handles for WiFi Task */
TaskHandle_t wifi_task_handle;

/* Maintains the connection id of the current connection */
uint16_t conn_id = 0;

/* This variable is set to true when button callback is received and
 * data is present in EMEEPROM. It is set to false after the WiFi Task
 * processes Disconnection notification. It is used to check button
 * interrupt while the device is trying to connect to WiFi
 */
bool button_pressed = false;

/******************************************************************************
 *                              Function Prototypes
 ******************************************************************************/
static wiced_result_t           app_management_callback(wiced_bt_management_evt_t event,
                                      wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t   app_gatts_callback(wiced_bt_gatt_evt_t event,
                                      wiced_bt_gatt_event_data_t *p_data);
void                            app_set_advertisement_data(void);
static void                     application_init(void);
static void                     gpio_interrupt_handler(void *handler_arg,
                                      cyhal_gpio_event_t event);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU
*    1. Initializes the BSP
*    2. Configures the button for interrupt
*    3. Initializes retarget IO for UART debug printing
*    4. Initializes platform configuration
*    5. Initializes BT stack and heap
*    6. Creates WiFi connect and disconnect tasks
*    7. Starts the RTOS scheduler
*
* Return:
*  int
*
*******************************************************************************/
int main()
{
    /* Return status for EEPROM. */
    cy_en_em_eeprom_status_t eepromReturnValue;

    /* Initialize the board support package */
    if(CY_RSLT_SUCCESS != cybsp_init())
    {
        CY_ASSERT(0);
    }

    /* Initialize the user button.*/
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                        CYHAL_GPIO_DRIVE_NONE, CYBSP_BTN_OFF);

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN,
                                 gpio_interrupt_handler, NULL);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    printf("***************************\n"
       "Wi-Fi Onboarding Using BLE\n"
       "***************************\n\n");

    /* Initialize the EMEEPROM. */
    eepromReturnValue = Cy_Em_EEPROM_Init(&Em_EEPROM_config, &Em_EEPROM_context);

    if(CY_EM_EEPROM_SUCCESS != eepromReturnValue)
    {
        printf("Error initializing EMEEPROM\n");
        CY_ASSERT(0);
    }

    /* Configure platform specific settings for Bluetooth */
    cybt_platform_config_init(&bt_platform_cfg_settings);

    /* Initialize the Bluetooth stack with a callback function and stack
     * configuration structure */
    if(WICED_SUCCESS != wiced_bt_stack_init (app_management_callback, &wiced_bt_cfg_settings))
    {
        printf("Error initializing BT stack\n");
        CY_ASSERT(0);
    }

    /* Initialize WiFi Tasks */
    xTaskCreate(wifi_task,
                "WiFi-Task",
                WIFI_TASK_STACK_SIZE,
                NULL,
                WIFI_TASK_PRIORITY,
                &wifi_task_handle);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* Should never get here */
    CY_ASSERT(0) ;
}


/*******************************************************************************
* Function Name: application_init
********************************************************************************
* Summary:
* This function is called from the BTM enabled event
*    1. Initializes and registers the GATT DB
*    2. Sets pairable mode to true
*    3. Sets ADV data and starts advertising
*
*******************************************************************************/
void application_init(void)
{
	/* Generate a random local Bluetooth Device Address and print it out */
	wiced_bt_device_address_t bda = {0};
	cyhal_trng_t trng_obj;
	cyhal_trng_init(&trng_obj);
	bda[0] = (uint8_t) cyhal_trng_generate(&trng_obj);
	bda[1] = (uint8_t) cyhal_trng_generate(&trng_obj);
	bda[2] = (uint8_t) cyhal_trng_generate(&trng_obj);
	bda[3] = (uint8_t) cyhal_trng_generate(&trng_obj);
	bda[4] = (uint8_t) cyhal_trng_generate(&trng_obj);
	bda[5] = (uint8_t) cyhal_trng_generate(&trng_obj);
	cyhal_trng_free(&trng_obj);
	wiced_bt_set_local_bdaddr( bda, BLE_ADDR_RANDOM);

	wiced_bt_gatt_status_t gatt_status;

    /* Return status for EEPROM. */
    cy_en_em_eeprom_status_t eepromReturnValue;

    /* Enabling button interrupt */
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            GPIO_INTERRUPT_PRIORITY, true);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_gatts_callback);
    printf("\nGATT status:\t");
    printf(get_bt_gatt_status_name(gatt_status));
    printf("\n");

    /*  Inform the stack to use our GATT database */
    gatt_status =  wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, false);

    /* Set the advertising params and make the device discoverable */
    app_set_advertisement_data();

    /* Determine if the EEPROM has a previously stored WiFi SSID value.
     * If it does, use the stored credentials to connect to WiFi.
     * Otherwise, start BLE advertisements so that the user can use BLE
     * to enter the WiFi credentials. */
    eepromReturnValue = Cy_Em_EEPROM_Read(LOGICAL_EEPROM_START, (void *)&wifi_details.wifi_ssid[0],
                                          sizeof(wifi_details), &Em_EEPROM_context);

    if((CY_EM_EEPROM_SUCCESS == eepromReturnValue) && (0 != wifi_details.ssid_len))
    {
        printf("Data present in EMEEPROM\n");
        /* Unblock WiFi task with notification value indicating the WiFi credentials
         * should be taken from EMEEPROM
         */
        xTaskNotify(wifi_task_handle, NOTIF_EMEEPROM, eSetValueWithOverwrite);
    }
    else /* WiFi credentials not found in EMEEPROM */
    {
        printf("Data not present in EMEEPROM\n");
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW,
                                            BLE_ADDR_PUBLIC, NULL);
    }
}

/*******************************************************************************
* Function Name: app_set_advertisement_data
********************************************************************************
* Summary:
* This function sets the advertising data.
*
*******************************************************************************/
void app_set_advertisement_data(void)
{
    wiced_result_t result;

    wiced_bt_ble_advert_elem_t adv_elem[ADV_ELEMENTS];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t custom_service_uuid[LEN_UUID_128] = {__UUID_SERVICE_CUSTOM_SERVICE};

    /* First element is the advertisment flags */
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    /* Second element is the 128 bit custom UUID */
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = custom_service_uuid;
    num_elem++;

    /* Final element is the device name */
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = app_gap_device_name_len;
    adv_elem[num_elem].p_data       = (uint8_t *)app_gap_device_name;
    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
    if(WICED_SUCCESS != result)
    {
        printf("Set ADV data failed\n");
    }
}


/*******************************************************************************
* Function Name: app_management_callback
********************************************************************************
* Summary:
* This function handles the BT stack events.
*
* Parameters:
*  wiced_bt_management_evt_t: event code
*  p_event_data: Pointer to the event data
*
* Return:
*  wiced_result_t: Result
*
*******************************************************************************/
wiced_result_t app_management_callback(wiced_bt_management_evt_t event,
                                   wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                      result = WICED_BT_SUCCESS;

    printf("Bluetooth Management Event: \t");
    printf(get_bt_event_name(event));
    printf("\n");

    switch(event)
    {
    case BTM_ENABLED_EVT:
        /* Initialize the application */
        application_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap =
                                                    BTM_IO_CAPABILITIES_NONE;

        p_event_data->pairing_io_capabilities_ble_request.oob_data =
                                                                BTM_OOB_NONE;

        p_event_data->pairing_io_capabilities_ble_request.auth_req =
                                                            BTM_LE_AUTH_REQ_SC;

        p_event_data->pairing_io_capabilities_ble_request.max_key_size = MAX_KEY_SIZE;

        p_event_data->pairing_io_capabilities_ble_request.init_keys =
                                                        BTM_LE_KEY_PENC |
                                                        BTM_LE_KEY_PID |
                                                        BTM_LE_KEY_PCSRK |
                                                        BTM_LE_KEY_LENC;

        p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                                                        BTM_LE_KEY_PENC|
                                                        BTM_LE_KEY_PID|
                                                        BTM_LE_KEY_PCSRK|
                                                        BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        if(WICED_SUCCESS == p_event_data->pairing_complete.pairing_complete_info.ble.status)
        {
            printf("Pairing Complete: SUCCESS\n");
        }
        else /* Pairing Failed */
        {
            printf("Pairing Complete: FAILED\n");
        }
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* Paired Device Link Keys update */
        result = WICED_SUCCESS;
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        result = WICED_BT_ERROR;
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* Local identity Keys Update */
        result = WICED_SUCCESS;
        break;

    case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* Local identity Keys Request */
        result = WICED_BT_ERROR;
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        if(WICED_SUCCESS == p_event_data->encryption_status.result)
        {
            printf("Encryption Status Event: SUCCESS\n");
        }
        else /* Encryption Failed */
        {
            printf("Encryption Status Event: FAILED\n");
        }
    break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                                        WICED_BT_SUCCESS);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        printf("\n");
        printf("Advertisement state changed to ");
        printf(get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
        printf("\n");
        break;

    default:
        break;
    }

    return result;
}

/*******************************************************************************
* Function Name: app_get_attribute
********************************************************************************
* Summary:
* This function searches through the GATT DB to point to the attribute
* corresponding to the given handle
*
* Parameters:
*  uint16_t handle: Handle to search for in the GATT DB
*
* Return:
*  gatt_db_lookup_table_t *: Pointer to the correct attribute in the GATT DB
*
*******************************************************************************/
gatt_db_lookup_table_t * app_get_attribute(uint16_t handle)
{
    /* Search for the given handle in the GATT DB and return the pointer to the
    correct attribute */
    uint8_t array_index = 0;

    for (array_index = 0; array_index < app_gatt_db_ext_attr_tbl_size; array_index++)
    {
        if (app_gatt_db_ext_attr_tbl[array_index].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[array_index]);
        }
    }
    return NULL;
}

/*******************************************************************************
* Function Name: app_gatts_req_read_handler
********************************************************************************
* Summary:
* This function handles the GATT read request events from the stack
*
* Parameters:
*  uint16_t conn_id: Connection ID
*  wiced_bt_gatt_read_t * p_read_data: Read data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_read_handler(uint16_t conn_id,
                                             wiced_bt_gatt_read_t * p_read_data)
{

    gatt_db_lookup_table_t *puAttribute;
    int attr_len_to_copy;

    /* Get the right address for the handle in Gatt DB */
    if (NULL == (puAttribute = app_get_attribute(p_read_data->handle)))
    {
        printf("Read handle attribute not found. Handle:0x%X\n", p_read_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;

    printf("GATT Read handler: handle:0x%X, len:%d\n",
           p_read_data->handle, attr_len_to_copy);

    /* If the incoming offset is greater than the current length in the GATT DB
    then the data cannot be read back*/
    if (p_read_data->offset >= puAttribute->cur_len)
    {
        attr_len_to_copy = 0;
    }

    /* Calculate the number of bytes and the position of the data and copy it to
     the given pointer */
    if (attr_len_to_copy != 0)
    {
        uint8_t *from;
        int size_to_copy = attr_len_to_copy - p_read_data->offset;

        if (size_to_copy > *p_read_data->p_val_len)
        {
            size_to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_data) + p_read_data->offset;
        *p_read_data->p_val_len = size_to_copy;

        memcpy(p_read_data->p_val, from, size_to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: app_gatts_req_write_handler
********************************************************************************
* Summary:
* This function handles the GATT write request events from the stack
*
* Parameters:
*  uint16_t conn_id: Connection ID
*  wiced_bt_gatt_write_t * p_data: Write data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_write_handler(uint16_t conn_id,
                                                 wiced_bt_gatt_write_t * p_data)
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;
    gatt_db_lookup_table_t *puAttribute;

    printf("GATT write handler: handle:0x%X len:%d\n",
        p_data->handle, p_data->val_len);

    /* Get the right address for the handle in Gatt DB */
    if (NULL == (puAttribute = app_get_attribute(p_data->handle)))
    {
        printf("\nWrite Handle attr not found. Handle:0x%X\n", p_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    switch (p_data->handle)
    {
    /* Write request for the WiFi SSID characteristic. Copy the incoming data
    to the WIFI SSID variable */
    case HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE:
        memset(app_custom_service_wifi_ssid, 0, strlen((char *)app_custom_service_wifi_ssid));
        memcpy(app_custom_service_wifi_ssid, p_attr, p_data->val_len);
        puAttribute->cur_len = p_data->val_len;
        printf("Wi-Fi SSID: %s\n", app_custom_service_wifi_ssid);
        break;

    /* Write request for the WiFi password characteristic. Accept the password.
    Copy the incoming data to the WIFI Password variable */
    case HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE:
        memset(app_custom_service_wifi_password, 0, strlen((char *)app_custom_service_wifi_password));
        memcpy(app_custom_service_wifi_password, p_attr, p_data->val_len);
        puAttribute->cur_len = p_data->val_len;
        printf("Wi-Fi Password: %s\n", app_custom_service_wifi_password);
        break;

    /* Write request for the connection characteristic. Copy the incoming data
    to the WIFI connection variable. Based on the value either start connect
    or disconnect procedure */
    case HDLC_CUSTOM_SERVICE_WIFI_CONNECTION_VALUE:
        app_custom_service_wifi_connection[0] = p_attr[0];
        puAttribute->cur_len = p_data->val_len;

        /* Connection message */
        if(app_custom_service_wifi_connection[0])
        {
            /* Unblock WiFiConnect task with notification value indicating the WiFi credentials
                * should be taken from GATT DB
                */
            xTaskNotify(wifi_task_handle, NOTIF_GATT_DB, eSetValueWithOverwrite);
        }
        else /* Disconnection message */
        {
            xTaskNotify(wifi_task_handle, NOTIF_DISCONNECT_GATT_DB, eSetValueWithOverwrite);
        }
        break;

    /* Notification for connection characteristic. If enabled, notification can
        * be sent to the client if the connection was successful or not
        */
    case HDLD_CUSTOM_SERVICE_WIFI_CONNECTION_CLIENT_CHAR_CONFIG:
        app_custom_service_wifi_connection_client_char_config[0] = p_attr[0];
        app_custom_service_wifi_connection_client_char_config[1] = p_attr[1];
        break;

    default:
        printf("Write GATT Handle not found\n");
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    return  result;
}


/*******************************************************************************
* Function Name: app_gatt_connect_callback
********************************************************************************
* Summary:
* This function handles the GATT connect request events from the stack
*
* Parameters:
*  wiced_bt_gatt_connection_status_t *p_conn_status: Connection or disconnection
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatt_connect_callback(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    /* Check whether it is a connect event or disconnect event. If the device
    has been disconnected then restart advertisement */
    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
            /* Device got connected */
            printf("\nConnected: Peer BD Address: ");
            print_bd_address(p_conn_status->bd_addr);
            printf("\n");
            conn_id = p_conn_status->conn_id;
        }
        else /* Device got disconnected */
        {
            printf("\nDisconnected: Peer BD Address: ");
            print_bd_address(p_conn_status->bd_addr);
            printf("\n");

            printf("Reason for disconnection: \t");
            printf(get_bt_gatt_disconn_reason_name(p_conn_status->reason));
            printf("\n");

            conn_id = 0;

            /* Set the advertising params and make the device discoverable */
            app_set_advertisement_data();

            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/*******************************************************************************
* Function Name: app_gatts_req_cb
********************************************************************************
* Summary:
* This function redirects the GATT attribute requests to the appropriate functions
*
* Parameters:
*  wiced_bt_gatt_attribute_request_t *p_data: GATT request data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_cb(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = app_gatts_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = app_gatts_req_write_handler(p_data->conn_id, &(p_data->data.write_req));
        break;

    case GATTS_REQ_TYPE_MTU:
        printf("Exchanged MTU from client: %d\n", p_data->data.mtu);
        result = WICED_BT_GATT_SUCCESS;
        break;

    default:
        break;
    }

    return result;
}

/*******************************************************************************
* Function Name: app_gatts_callback
********************************************************************************
* Summary:
* This function redirects the GATT requests to the appropriate functions
*
* Parameters:
*  wiced_bt_gatt_attribute_request_t *p_data: GATT request data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event,
                                             wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = app_gatt_connect_callback(&p_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = app_gatts_req_cb(&p_data->attribute_request);
        break;

    default:
        break;
    }
    return result;
}

/*******************************************************************************
 * Function Name: gpio_interrupt_handler
 *******************************************************************************
 * Summary:
 *  GPIO interrupt service routine. This function detects button presses
 *  and passes a notification to the WiFi task to disconnect from WiFi and
 *  delete EMEEPROM data.
 *
 *
 * Parameters:
 *  void *callback_arg : pointer to variable passed to the ISR
 *  cyhal_gpio_event_t event : GPIO event type
 *
 *******************************************************************************/
void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
     /* Notify the WiFi task to disconnect */
     button_pressed = true;
     xTaskNotifyFromISR(wifi_task_handle, NOTIF_DISCONNECT_BTN, eSetValueWithOverwrite, pdFALSE);
}


/* [] END OF FILE */
