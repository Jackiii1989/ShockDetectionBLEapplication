#ifndef BLE_CUS_H__
#define BLE_CUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "MacroDefinitions.h"
#include "steval_MKI1178V1_central.h"
#include "ble_db_discovery.h"
/**@brief   Macro for defining a ble_hrs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CUS_DEF(_name)                                                                          \
static ble_cus_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_cus_on_ble_evt, &_name)


#define CUSTOM_PERIPH_SERVICE_UUID_BASE  {0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, \
                                          0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}


#define CUS_BASE_UUID                   {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x8E}} /**< Used vendor specific UUID. */

#define BLE_UUID_CUS_SERVICE            0x0001                      /**< The UUID of the Nordic UART Service. */



//#define SHOCK_DETECTION_SERVICE_UUID      0x1400

#define CUSTOM_VALUE_CHAR_UUID            0x1402
#define ACCEL_DT_CHAR_UUID                0x1403
#define ACCEL_INTEG_CHAR_UUID             0x1404
#define ACCEL_PEAK_CHAR_UUID              0x1405



/**@brief FLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa
 *        and an 8-bit exponent. */
typedef struct
{
  int8_t  exponent;                                                         /**< Base 10 exponent */
  int32_t mantissa;                                                         /**< Mantissa, should be using only 24 bits */
} ieee_float32_t;

																				
/**@brief Custom Service event type. */
typedef enum
{
        
    BLE_CUS_EVT_DISCOVERY_COMPLETE,
    BLE_CUS_EVT_DISCONNECTED,
    BLE_CUS_C_EVT_NUS_TX_EVT,
} ble_cus_evt_type_t;



/**@brief   Structure containing the handles related to the Running Speed and Cadence Service found on the peer. */
typedef struct
{
    uint16_t peak_cccd_handle;                /**< Handle of the CCCD of the peak characteristic*/
    uint16_t peak_handle;                     /**< Handle of the peak value characteristic */
    uint16_t sampling_time_cccd_handle;       /**< Handle of the CCCD of the sampling time characteristic */
    uint16_t sampling_time_handle;            /**< Handle of the sampling time value characteristic*/
    uint16_t integ_cccd_handle;               /**< Handle of the CCCD of the integration characteristic*/
    uint16_t integ_handle;                    /**< Handle of the integration vlaue characteristic. */
} ble_cus_c_db_t;


/**@brief   Structure containing the the data of th shock detection measurment. */
typedef struct
{
   
    uint8_t custom_value[5];                     /**<Custom Value. */
    uint16_t custom_value_len;

    uint8_t peak[5];                             /**< Peak of Impulse. */
    uint16_t peak_value_len;

    uint8_t integration_value[5];                /**< Integration value */
    uint16_t integration_value_len;

    uint8_t sampling_time[4];                     /**< time of the pulse */
    uint16_t sampling_time_value_len;
} ble_cus_c_t;


/**@brief Custom Service event. */
typedef struct
{
    uint16_t         conn_handle;        /**< Connection handle. */
    ble_cus_evt_type_t evt_type;         /**< Type of event. */
     union
    {
        ble_cus_c_db_t cus_db;           /**< Central unifed structure related handles found on the peer device. This will be filled if the evt_type is @ref BLE_RSCS_C_EVT_DISCOVERY_COMPLETE.*/
        ble_cus_c_t      cus;            /**< Pointer to the data This will be filled if the evt_type is @ref BLE_RSCS_C_EVT_RSC_NOTIFICATION. */
    } params;
} ble_cus_evt_t;


// Forward declaration of the ble_cus_t type.
typedef struct ble_cus_s ble_cus_t;


/**@brief Custom Service event handler type. */
typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_bas, ble_cus_evt_t * p_evt);


/**@brief  init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Central unifed structure. */
    //uint8_t                       initial_custom_value;           /**< Initial custom value */
    //ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_init_t;


/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_cus_s
{
    uint8_t                       uuid_type;                      /**< Definition of the UUID type: is it already defined or is it defined by the user */   
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */   
    ble_cus_c_db_t cus_db;                                        /**< Central unifed structure related handles found on the peer device. This will be filled if the evt_type is @ref BLE_RSCS_C_EVT_DISCOVERY_COMPLETE.*/
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    
};

/**@brief BLE CUS measurement values. This contains information on how to save the float whihc will be then parsed to the SDK stack. */
/*typedef struct ble_cus_meas_s
{
   ieee_float32_t                value_in_float;                           /**< Temperature Measurement Value (Celcius). */
//} ble_cus_meas_t;


/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_cus      Custom Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);



/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_bas          Custom Service structure.
 * @param[in]   Custom value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */


/**@brief Function for handling events from the database discovery module.
 *
 * @details This function will handle an event from the database discovery module, and determine
 *          if it relates to the discovery of NUS at the peer. If so, it will
 *          call the application's event handler indicating that NUS has been
 *          discovered at the peer. It also populates the event with the service related
 *          information before providing it to the application.
 *
 * @param[in] p_ble_nus_c Pointer to the NUS client structure.
 * @param[in] p_evt       Pointer to the event received from the database discovery module.
 */
 void ble_cus_c_on_db_disc_evt(ble_cus_t* p_ble_cus_c, ble_db_discovery_evt_t* p_evt);


/**@brief Function for assigning handles to a this instance of nus_c.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate this link to this instance of the module. This makes it
 *          possible to handle several link and associate each link to a particular
 *          instance of this module. The connection handle and attribute handles will be
 *          provided from the discovery event @ref BLE_NUS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_nus_c    Pointer to the NUS client structure instance to associate with these
 *                           handles.
 * @param[in] conn_handle    Connection handle to associated with the given NUS Instance.
 * @param[in] p_peer_handles Attribute handles on the NUS server that you want this NUS client to
 *                           interact with.
 *
 * @retval    NRF_SUCCESS    If the operation was successful.
 * @retval    NRF_ERROR_NULL If a p_nus was a NULL pointer.
 */
uint32_t ble_cus_c_handles_assign(ble_cus_t*                  p_ble_cus,
                                  uint16_t                    conn_handle,
                                  const ble_cus_c_db_t*    p_db_handles);




//uint32_t ble_cus_custom_value_update(ble_cus_t * p_cus, uint8_t custom_value);
//uint32_t ble_cus_dt_value_update(ble_cus_t * p_cus, float dt);
//uint32_t ble_cus_integ_value_update(ble_cus_t * p_cus, float integ);
//uint32_t ble_cus_peak_value_update(ble_cus_t * p_cus, float peak);


/**@brief Function handle value X (hvx),where X symbolize either notification of indication as the 
 *        struct and function can be used for both.         
 *
 * @details This function handles the details when a notification is send from the peripheral side and
 *          the needed discovery of the BLE table needs to be discovered.  
 *
 *
 * @param[in] p_ble_cus_c    Pointer to the CUS client structure instance to associate with these
 *                           handles.
 * @param[in] p_ble_evt      Connection handle to associated with the given NUS Instance.
 *
 * @retval    NRF_SUCCESS    If the operation was successful.
 * @retval    NRF_ERROR_NULL If a p_nus was a NULL pointer.
 */
static void on_hvx(ble_cus_t * p_ble_cus_c, ble_evt_t const * p_ble_evt);


/**@brief initialize the and assign the the whole Bluettooth table to the SDk stack         
 *
 * @details In order to work the the internal structure in BLE table needs to be initialized.
 *          The Profile of the BLE needs to be privded with values, like Service, Charakteristic,
 *          read and write rights and etc.
 *
 *
 * @param[in] m_cus_c        structure which be initialize and used for defining the
 *                           BLE table. 
 * @param[in] init           initial value that will be used for initialization
 * @retval    NRF_SUCCESS    If the operation was successful.
 */
uint32_t ble_cus_init(ble_cus_t *m_cus_c, const ble_cus_init_t* init);


//static uint8_t ble_measurement_encode(ble_cus_meas_t* p_cus_meas, uint8_t* p_encoded_buffer);


/**@brief Enable notification for the characteristic        
 *
 * @details In order to not get an notification when the peripheral has a new value, the 
 *          notification on central side needs also to be enabled.
 *
 *
 * @param[in]   conn_handle     connection handle for the BLE communication
 * @param[in]   cccd_handle     the handle number to the charachteristic
 * @param[in]   enable          enabling and dissabling the notification

 * @retval    NRF_SUCCESS    If the operation was successful.
 */
uint32_t ble_cus_notif_enable(uint16_t conn_handle,  uint16_t cccd_handle, bool enable);


#endif // BLE_CUS_H__
