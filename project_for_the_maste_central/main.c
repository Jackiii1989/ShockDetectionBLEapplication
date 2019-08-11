/**
 * Copyright (c) 2019, Aljosa Klajderic
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"


#include "ble_nus_c.h"
#include "ble_cus_central.h"

#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "my_log.h"


// enabling the debug logger and which logger to be used
#define MY_RTT 1
#define DEBUG_LOG 0
#if MY_RTT 
  #define LOG MY_LOG
#else
  #define LOG NRF_LOG_RAW_INFO
#endif


#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
                                   
BLE_CUS_DEF(m_cus_c);                                                     /**< BLE Nordic central unified Service (CUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                 /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                          /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                                 /**< Scanning Module instance. */

static uint16_t g_init_char_notify;                                       /**< Global flag to unify the values. */

// storing the local values of the transfered data over bleutooth 
static float sampling_time_value;
static float integration_value;
static float peak_value;

 /**@brief CUS UUID. */
    ble_uuid_t const m_cus_uuid_periph =
    {
      .uuid = BLE_UUID_NUS_SERVICE,
      .type = BLE_UUID_TYPE_VENDOR_BEGIN
    };

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}



/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_cus_c_on_db_disc_evt(&m_cus_c, p_evt);
}

//-------------------------------------------------------------------------------------------------------
// scan module function

/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             LOG("Connecting to target %02x%02x%02x%02x%02x%02x\n",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );


         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             scan_start();
         } break;

         default:
             break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;


    // set the filter parameters
    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_cus_uuid_periph);
    APP_ERROR_CHECK(err_code);

    bool MatchAllFilter = false;
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, MatchAllFilter);
    APP_ERROR_CHECK(err_code);
}


//-------------------------------------------------------------------------------------------------------





/**@brief converting the raw bytes to the appropriate data types for integration and peak values
 *
 * @param[in]   value_to_save   variable to save the data
 * @param[in]   raw_bytes       raw bytes trasnmitted from BLE
 * @param[in]   raw_bytes       raw bytes trasnmitted from BLE
 */

void convert_to_float(float* value_to_save, uint8_t const* raw_bytes, unsigned int converting_factor)
{

  int mantissa = {0};
  memcpy( &mantissa, raw_bytes, sizeof(int) -1 );
#if DEBUG_LOG  
  LOG("\n int value: %x \n", mantissa); 
#endif
  if (mantissa >= 0x800000 ) { // if the value is negative
          mantissa = -((0xFFFFFF + 1) - mantissa);
  }
#if DEBUG_LOG
  LOG(" int value: %x \n", mantissa);
  LOG(" int value: %d \n\n", mantissa);
#endif
  *value_to_save = ((float)mantissa /converting_factor);

}

/**@brief converting the raw bytes to the appropriate data types for sampling time
 *
 * @param[in]   value_to_save   variable to save the data
 * @param[in]   raw_bytes       raw bytes trasnmitted from BLE
 * @param[in]   raw_bytes       raw bytes trasnmitted from BLE
 */

void convert_to_float_dt(float* value_to_save, uint8_t const* raw_bytes, unsigned int converting_factor)
{

  int mantissa = {0};
  memcpy( &mantissa, raw_bytes, sizeof(int) -1 );
#if DEBUG_LOG
  
  LOG( "raw values dt: %x %x %x %x,\r\n", raw_bytes[0],  raw_bytes[1], raw_bytes[2],  raw_bytes[3]); 
  LOG("int value dt: %d \n\n", mantissa);
#endif
  *value_to_save = ((float)mantissa /converting_factor);

}


static void ble_cus_c_evt_handler(ble_cus_t * p_ble_cus_c, ble_cus_evt_t const * p_ble_cus_evt)
{
    ret_code_t err_code = {0};

    switch (p_ble_cus_evt->evt_type)
    {
        case BLE_CUS_EVT_DISCOVERY_COMPLETE:

            VERIFY_PARAM_NOT_NULL(p_ble_cus_c);
            err_code = ble_cus_c_handles_assign(p_ble_cus_c, p_ble_cus_evt->conn_handle, &p_ble_cus_evt->params.cus_db); 
            APP_ERROR_CHECK(err_code);

            // activate notification: when new data is available send it to the central
            err_code = ble_cus_notif_enable(p_ble_cus_c->conn_handle, p_ble_cus_c->cus_db.integ_cccd_handle, true);
            APP_ERROR_CHECK(err_code);
            g_init_char_notify = 1;
            break;
      
        case BLE_CUS_C_EVT_NUS_TX_EVT:
          
            // convert the raw data into the appropriate type          
            convert_to_float(&peak_value, p_ble_cus_evt->params.cus.peak, 1000);
            convert_to_float(&integration_value, p_ble_cus_evt->params.cus.integration_value, 1000);
            convert_to_float_dt(&sampling_time_value, p_ble_cus_evt->params.cus.sampling_time, 100000);
            
            char dataMsg[128] = {0};
            sprintf(dataMsg,"\n sampling time:\t\t" NRF_LOG_FLOAT_MARKER
                            ",\n integration value:\t" NRF_LOG_FLOAT_MARKER
                            ",\n peak: \t\t\t" NRF_LOG_FLOAT_MARKER "\n", 
                            NRF_LOG_FLOAT(sampling_time_value),
                            NRF_LOG_FLOAT(integration_value),
                            NRF_LOG_FLOAT(peak_value));

            SEGGER_RTT_WriteString(0, dataMsg);
            break;

        case BLE_CUS_EVT_DISCONNECTED:
            LOG("Disconnected.\n");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            // Discover peer's services: start discovery of the service. The Client waits for a discovery result.
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            LOG("Disconnected. conn_handle: 0x%x, reason: 0x%x\n",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                LOG("Connection Request timed out.\n");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);

            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            LOG("PHY update request.\n");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        }   break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            LOG("GATT Client Timeout.\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            LOG("GATT Server Timeout.\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            
            // notify goes into to steps (we have 3 Charachteristic that is why we have 3 step notify init)
            if (g_init_char_notify == 1)
            {
              err_code = ble_cus_notif_enable(m_cus_c.conn_handle, m_cus_c.cus_db.peak_cccd_handle, true);
              APP_ERROR_CHECK(err_code);
              g_init_char_notify =2;
              
            }
            else if (g_init_char_notify == 2){
                        
              err_code = ble_cus_notif_enable(m_cus_c.conn_handle, m_cus_c.cus_db.sampling_time_cccd_handle, true);
              APP_ERROR_CHECK(err_code);
              g_init_char_notify =3;
            
            }
            break;

        default:
            break;
    }
}

/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    // send a request to the sdk stack to change the clock configuration
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_cus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void cus_c_init(void)
{
    ret_code_t          err_code;
    ble_cus_init_t      init = {0};
    init.evt_handler  = ble_cus_c_evt_handler;

    err_code = ble_cus_init(&m_cus_c, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


int main(void)
{

    // Initialize.
    log_init();
    timer_init();
    power_management_init();

    buttons_leds_init();
    db_discovery_init(); 
    
    ble_stack_init();
    gatt_init();
    cus_c_init();
    scan_init();

    // Start execution.
    LOG("BLE central example started.\n");
    scan_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
