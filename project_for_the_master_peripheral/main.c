
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "MacroDefinitions.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "steval_MKI1178V1.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_cus.h"

#include "my_log.h"

#define MY_RTT 1
#define DEBUG_LOG 0
#if MY_RTT 
  #define LOG MY_LOG
#else
  #define LOG NRF_LOG_RAW_INFO
#endif


#define APP_BLE_CONN_CFG_TAG            1                                      /**< A tag identifying the SoftDevice BLE configuration. */
#define DEVICE_NAME                     "Master"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "PTConnected"                          /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                      /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL                64                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                54000                                   /**< The advertising duration (540 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(8000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define NOTIFICATION_INTERVAL           APP_TIMER_TICKS(1000)     


// security bond parameters
#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  1                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
                                    
static uint8_t STAT_REG_ACCEL  = 0 ;                                            /**< intern status register for handling the intern events of the accelerometer */

// intern saved register values from the accelerometer  

#define STAT_REG_ACCEL_START_SAMPLING         (0x01)                            /**< sampling started  */
#define STAT_REG_ACCEL_FIFO_FULL              (0x02)                            /**< fifo full         */
#define STAT_REG_ACCEL_FINISH_SAMPLING        (0x04)                            /**< sampling finished */
#define STAT_REG_ACCEL_ST_CALC                (0x08)                            /**< start the calculation of the data */
#define STAT_REG_ACCEL_START_SENDING          (0x10)                            /**< calculation finished */
#define STAT_REG_ACCEL_CYCLE_FINISHED         (0x20)                            /**< calculation finished */

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/                                                          
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */
BLE_CUS_DEF(m_pus);  
APP_TIMER_DEF(m_notification_timer_id);

static uint8_t m_custom_value = 0;

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */


// UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_PUS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN }
};


static void advertising_start(bool erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;


    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {

#if DEBUG_LOG
            LOG("pm_evt_handler:Connected to a previously bonded device,PM_EVT_BONDED_PEER_CONNECTED.\n");
#endif
        } break;


        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        {

#if DEBUG_LOG
            LOG("pm_evt_handler:PM_EVT_LOCAL_DB_CACHE_APPLIED.\n");
#endif
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
#if DEBUG_LOG
            LOG("pm_evt_handler:PM_EVT_PEER_DATA_UPDATE_SUCCEEDED.\n");
#endif
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
           /*
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
            */
            
            LOG("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d. \n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);

            LOG( "-------------------------------------------------------------------------------------\n");
            
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
#if DEBUG_LOG
            LOG("pm_evt_handler:p_evt->evt_id: 0x%x.\n",p_evt->evt_id);
#endif
            break;
    }
}

static void notification_timeout_handler(void * p_context);

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module. --> basically queue parameter are init
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers. --> basically checks the parameter and copies them to m_notification_timer_id
    err_code = app_timer_create(&m_notification_timer_id, APP_TIMER_MODE_SINGLE_SHOT, notification_timeout_handler);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
   
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;

    // the connection interval --> data to be skipped
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;


    // sets GAP Peripheral Connection Parameters
    // does the frequency -hopping scheme
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        //LOG("GATT ATT MTU on connection 0x%x changed to %d.\n",p_evt->conn_handle, p_evt->params.att_mtu_effective);
    }

    //ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
        ret_code_t          err_code;
        nrf_ble_qwr_init_t  qwr_init = {0};
        ble_pus_init_t      pus_init = {0};

        // Initialize Queued Write Module.
        qwr_init.error_handler = nrf_qwr_error_handler;

        err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
        APP_ERROR_CHECK(err_code);

         // Initialize CUS Service init structure to zero.
        pus_init.evt_handler                = NULL;//on_cus_evt;
    
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&pus_init.custom_value_char_attr_md.cccd_write_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&pus_init.custom_value_char_attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&pus_init.custom_value_char_attr_md.write_perm);
    
        err_code = ble_pus_init(&m_pus, &pus_init);
        APP_ERROR_CHECK(err_code);
                
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
#if DEBUG_LOG
            LOG("on_conn_params_evt:BLE_CONN_PARAMS_EVT_FAILED.\n");
#endif
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

#if DEBUG_LOG
            LOG("entering sleep mode.\n");
#endif

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
#ifdef DEBUG_NRF
    //err_code = sd_power_system_off();
    //while(1);
#else
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //NRF_LOG_DEBUG("Fast advertising.");
#if DEBUG_LOG
            LOG("Fast advertising.\n");
#endif
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
#if DEBUG_LOG
            LOG("idle mode.\n");
#endif
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            LOG("Disconnected.\n");
            break;

        case BLE_GAP_EVT_CONNECTED:
            //NRF_LOG_DEBUG("Connected.");
            LOG("Connected.\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            LOG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            LOG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            LOG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            LOG("ble_evt_handler: BLE_GATTS_EVT_SYS_ATTR_MISSING\n");
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;


        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            //LOG("ble_evt_handler: BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST\n");
            //err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:  //Advertising report: meaing that we got an an information about an advertiser
            //LOG("ble_evt_handler: BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST\n");
            //err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE: //this case was considered to send higher volumes of the data packets to reduce overhead 
            //LOG("BLE_GAP_EVT_DATA_LENGTH_UPDATE was issued.\n");
            //err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
        default:
#if DEBUG_LOG
            //LOG("ble_evt_handler: 0x%x.\n",p_ble_evt->header.evt_id);
#endif
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
     A Softdevice must be enabled witht he required configuration parameters before it can be used
  Note: When setting the connection-specifiv using sde_ble_cfg_set(), you must create a tag for each configuration.
  This tag must be supplied when calling sde_ble_gap_adv_start() and sde_ble_gap_connect(). IF your application uses
  the Advertising Module, you must call ble_advertising_conn_cfg_tag_set before stating advertising.


 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    /*
       FUnction for requesting to enable the Softdevice:

       This function isssues NRF_SDH_EVT_ENABLE_REQUEST request to all observers that were registered using the 
       NRD_SDH_REQUEST_OBSERVER macro. The observers may or may not acknowledge the request. If all observers 
       acknowledge the request, the SoftDevice will be enableed. Otherwise, the process will be stopped and the
       oberserver that did not acknowledge have the responsibility to restart it by calling nrf_request_continue 
       when they are ready for the SOftdevice to change state. 

        Notify observer that a process will hapeen --> low clock configuration (observer pattern)
    */
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    /*
       this function configures the BLE stack witht he settings specified int he SOftdevice handler. The following
       configuration will be set:
       number of peripheral links, Number of central links, GATTS Attribute table size, Vendor specific UUID count
      @param[out] p_ram_start  Application RAm start address.     
    */
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    // the function also checks if the RAM address maches the application and if it has enough memory 
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register an observer handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief: Function for the Peer Manager initialization. Peer Manager API exposes functions for managing link 
/*         link security (initial pairing and encryption) for managing peers (manipulate associated with bonded peers)
           and for managing whitelist. 
/*          
/*

 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init(); // peer manager init
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_DEBUG("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
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


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }

}

void gpio_init(){

    ret_code_t err_code;
    // check if the gpio are already confgiured
    if(!nrf_drv_gpiote_is_init())
    {
      err_code = nrf_drv_gpiote_init();
      APP_ERROR_CHECK(err_code);
    }

    // output configuration
    nrf_drv_gpiote_out_config_t config_out;
    config_out.init_state =NRF_GPIOTE_INITIAL_VALUE_LOW;
    config_out.task_pin = false;
    APP_ERROR_CHECK(err_code);

    // config the ouptu pins
    err_code = nrf_drv_gpiote_out_init(STEVAL_MKI1178V1_SA0_PIN, &config_out); //pins 25
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(STEVAL_MKI1178V1_CS_PIN, &config_out); //pins 24
    APP_ERROR_CHECK(err_code);
       err_code = nrf_drv_gpiote_out_init(LED_2, &config_out);
    APP_ERROR_CHECK(err_code);

    // set them to high
    nrfx_gpiote_out_set(STEVAL_MKI1178V1_SA0_PIN); // pins 25
    nrfx_gpiote_out_set(STEVAL_MKI1178V1_CS_PIN); //pins 24
    nrfx_gpiote_out_set(LED_2);

    // input configuration --> creating a event when a low to high goes
    nrfx_gpiote_in_config_t  config_in = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    config_in.pull = NRF_GPIO_PIN_PULLUP;

    // init pin at STEVAL_MKI1178V1_INT1_PIN with configuration config_in and 
    // the callback function accel_callback
    err_code = nrfx_gpiote_in_init(STEVAL_MKI1178V1_INT1_PIN, &config_in, accel_callback);
    APP_ERROR_CHECK(err_code);

    // init pin at STEVAL_MKI1178V1_INT2_PIN with configuration config_in and 
    // the callback function accel_callback
    err_code = nrfx_gpiote_in_init(STEVAL_MKI1178V1_INT2_PIN, &config_in, accel_callback);
    APP_ERROR_CHECK(err_code);

}

static void accel_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    ret_code_t err_code;

    if(pin == STEVAL_MKI1178V1_INT1_PIN) //sensor did wake up -> start to sample the data
    {
       // check if in the status acceleration register STAT_REG_ACCEL the bit is not set, otherwise
       // if it is, then the a measurment is already running and something went wrong
       if ( (STAT_REG_ACCEL & STAT_REG_ACCEL_START_SAMPLING) != STAT_REG_ACCEL_START_SAMPLING)
       {
           STAT_REG_ACCEL |= STAT_REG_ACCEL_START_SAMPLING;

           MKI1178V1_get_triggered_axis();
           err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
           APP_ERROR_CHECK(err_code);
           nrf_drv_gpiote_out_toggle(LED_2);      
       }
    }

    if(pin == STEVAL_MKI1178V1_INT2_PIN) // buffer full, need to save data
    {
        MKI1178V1_save_data();
    }

}


static void notification_timeout_handler(void * p_context)
{

    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;

    err_code = app_timer_stop(m_notification_timer_id);
    APP_ERROR_CHECK(err_code);
    accel_t data_main = {0};
    processing(&data_main);

 #if DEBUG_LOG 
        MY_LOG( "\nsampling time:  " NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(data_main.integ_value));
        MY_LOG( "\tpeak:  " NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(data_main.peak));
        MY_LOG( "\tintegration:  " NRF_LOG_FLOAT_MARKER"\n ", NRF_LOG_FLOAT(data_main.sampling_time));
#endif 
        

     if(m_conn_handle != BLE_CONN_HANDLE_INVALID) // check if there is a valid communication handle
    {
  
      // update the 'persistent' system attribute information to the stack (allowed only for active connections)
       err_code = sd_ble_gatts_sys_attr_set(m_pus.conn_handle, NULL, 0, 0);
       APP_ERROR_CHECK(err_code);

      /* updating the Bluetooth table, which also means that we are sending the data to the master */
      err_code =ble_pus_sample_time_value_update(&m_pus, data_main.sampling_time);
      APP_ERROR_CHECK(err_code);
      
      err_code = ble_pus_integ_value_update(&m_pus, data_main.integ_value);
      APP_ERROR_CHECK(err_code);
      
      err_code = ble_pus_peak_value_update(&m_pus, data_main.peak);
      APP_ERROR_CHECK(err_code);

#if DEBUG_LOG
    LOG( "-------------------------------------------------------------------------------------\n");
#endif
  
    }
    else
    {
      LOG( "Communication Error:. Data won't be trasnported due to the missing connection to master.\n");
    }

    STAT_REG_ACCEL &= ~STAT_REG_ACCEL_START_SAMPLING;
    err_code = empty_the_fifo_Buffer();
    APP_ERROR_CHECK(err_code);
    put_in_active_mode();


}

/**@brief Function for application main entry.
 */
int main(void)

{
    bool erase_bonds;

    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    gpio_init();
    
    power_management_init();// you can use power managed if enabled
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

    // Start execution.
#if DEBUG_LOG
    LOG("Application BE  peripheral started.\n");
    
#endif;
    MKI1178V1_conf();
    uint8_t data;
    LOG("Threshold read from register in mg: %d.\n", MKI1178V1_read_treshold_in_mg(&data));
    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
          
         /* for the further consideration -> to save energy and monitor the mcu*/ 
        //idle_state_handle();
        __WFE();
    }
}



