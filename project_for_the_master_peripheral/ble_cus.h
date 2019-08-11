#ifndef BLE_CUS_H__
#define BLE_CUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "MacroDefinitions.h"
#include "steval_MKI1178V1.h"


/**@brief   Macro for defining a ble_hrs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CUS_DEF(_name)                                                                          \
static ble_pus_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_pus_on_ble_evt, &_name)






#define CUS_BASE_UUID                   {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                                        0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x8E}} /**< Used vendor specific UUID. */

#define BLE_UUID_PUS_SERVICE              0x0001                      /**< The UUID of the Nordic UART Service. */


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

																				
/**@brief peripheral Service event type. */
typedef enum
{
    BLE_PUS_EVT_NOTIFICATION_ENABLED,                             /**< peripheral value notification enabled event. */
    BLE_PUS_EVT_NOTIFICATION_DISABLED,                             /**< peripheral value notification disabled event. */
    BLE_PUS_EVT_DISCONNECTED,
    BLE_PUS_EVT_CONNECTED
} ble_pus_evt_type_t;

/**@brief Peripeheral Service event. */
typedef struct
{
    ble_pus_evt_type_t evt_type;                                  /**< Type of event. */
} ble_pus_evt_t;

// Forward declaration of the ble_cus_t type.
typedef struct ble_pus_s ble_pus_t;


/**@brief peripheral Service event handler type. */
typedef void (*ble_pus_evt_handler_t) (ble_pus_t * p_bas, ble_pus_evt_t * p_evt);

/**@brief peripheral Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_pus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_pus_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_pus_s
{
    ble_pus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      custom_value_handles;           /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      dt_value_handles;               /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      integ_value_handles;            /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      peak_value_handles;             /**< Handles related to the Custom Value characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
   
};

typedef struct ble_pus_meas_s
{
   ieee_float32_t                dt_value_in_float;                           /**< Temperature Measurement Value (Celcius). */
} ble_pus_meas_t;


/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_pus_init(ble_pus_t * p_pus, const ble_pus_init_t * p_cus_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_cus      Custom Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_pus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

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

//uint32_t ble_cus_custom_value_update(ble_cus_t * p_cus, uint8_t custom_value);
uint32_t ble_pus_sample_time_value_update(ble_pus_t * p_pus, float dt);
uint32_t ble_pus_integ_value_update(ble_pus_t * p_pus, float integ);
uint32_t ble_pus_peak_value_update(ble_pus_t * p_pus, float peak);
static uint8_t ble_measurement_encode(ble_pus_meas_t* p_pus_meas, uint8_t* p_encoded_buffer);
#endif // BLE_CUS_H__
