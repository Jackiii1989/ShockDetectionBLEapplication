#include "sdk_common.h"
#include "ble_cus.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "MacroDefinitions.h"

#define DEBUG_LOG 1

#if MY_RTT
  #include "my_log.h"
#else
  #define MY_LOG NRF_LOG_RAW_INFO
#endif

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_pus       Peripheral Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_pus_t * p_pus, ble_evt_t const * p_ble_evt)
{
    p_pus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_pus_evt_t evt;

    evt.evt_type = BLE_PUS_EVT_CONNECTED;

}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_pus       Peripheral Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_pus_t * p_pus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_pus->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    ble_pus_evt_t evt;

    evt.evt_type = BLE_PUS_EVT_DISCONNECTED;

    p_pus->evt_handler(p_pus, &evt);
}

/**@brief Function for receiving/senidng data from master/peripeheral
 *    
 * @detail this function is meant for communcation with master, where depending on the
 *         value some of the action should be done on. The value of data and action has not been
 *         decided. It was meant for further consideration.
 *
 * @param[in]   p_pus       peripheral Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_pus_t * p_pus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Custom Value Characteristic Written to.
    if (p_evt_write->handle == p_pus->custom_value_handles.value_handle)
    {
        nrf_gpio_pin_toggle(LED_4);
        MY_LOG("on_write: Event type = %x\r\n", p_ble_evt->header.evt_id);

        MY_LOG("on_write: p_evt_write->handle= %x \n", p_evt_write->handle );

        MY_LOG("on_write: p_pus->custom_value_handles.value_handle= %x \n", p_pus->custom_value_handles.value_handle);
        MY_LOG("on_write: *p_evt_write->data= %x \n", *p_evt_write->data);


    }

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_pus->custom_value_handles.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {
        MY_LOG("on_write: write handle->handle= %x, \
                p_cus->custom_value_handles.cccd_handle= %x \n",
                p_evt_write->handle, p_pus->custom_value_handles.cccd_handle);

        // CCCD written, call application event handler
        if (p_pus->evt_handler != NULL)
        {
            ble_pus_evt_t evt;


           MY_LOG("on_write: p_evt_write->data= %x\n",p_evt_write->data);
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_PUS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_PUS_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_pus->evt_handler(p_pus, &evt);
        }
    }

}


/**@brief Function for receiving/senidng data from master/peripeheral
 *    
 * @detail this function is meant for communcation with master, where depending on the
 *         value some of the action should be done on. The value of data and action has not been
 *         decided. It was meant for further consideration.
 *
 * @param[in]   p_context   Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */

void ble_pus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_pus_t * p_pus = (ble_pus_t *) p_context;
    
    
    if (p_pus == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
#if DEBUG_LOG
            //MY_LOG("ble_cus_on_ble_evt: BLE_GAP_EVT_CONNECTED: code %x\n",p_ble_evt->header.evt_id); 
#endif
            //void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
             p_pus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            //on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
#if DEBUG_LOG
            //MY_LOG("ble_cus_on_ble_evt: BLE_GAP_EVT_CONN_PARAM_UPDATE: code %x\n",p_ble_evt->header.evt_id); 
            MY_LOG("Peripheral ready to start!!!\n"); 
#endif
            break;

        case BLE_GAP_EVT_DISCONNECTED:
#if DEBUG_LOG
            MY_LOG("ble_pus_on_ble_evt: BLE_GAP_EVT_DISCONNECTED: code %x\n",p_ble_evt->header.evt_id); 
#endif
            //on_disconnect(p_cus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
#if DEBUG_LOG
            //MY_LOG("ble_pus_on_ble_evt: BLE_GATTS_EVT_WRITE: code %x\n",p_ble_evt->header.evt_id); 
#endif
            on_write(p_pus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
#if DEBUG_LOG
            //MY_LOG("ble_pus_on_ble_evt: BLE_GATTS_EVT_HVN_TX_COMPLETE: code %x\n",p_ble_evt->header.evt_id);
#endif
            break;

        default:
            // No implementation needed.
            //MY_LOG("ble_pus_on_ble_evt: Event number = %x\r\n", p_ble_evt->header.evt_id); 
            break;
    }
}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_pus        peripeheral unified structure.
 * @param[in]   p_pus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t custom_value_char_add(ble_pus_t * p_pus, const ble_pus_init_t * p_pus_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t initial_custom_value;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_pus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_pus->uuid_type;
    ble_uuid.uuid = CUSTOM_VALUE_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_pus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_pus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    initial_custom_value = 0;
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
     attr_char_value.p_value  = &initial_custom_value;

    err_code = sd_ble_gatts_characteristic_add(p_pus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_pus->custom_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);

//------------------------------------------------------------------------------------------------------------------------------
    // Add Custom Value characteristic dt
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_pus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_pus->uuid_type;
    ble_uuid.uuid = ACCEL_DT_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_pus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_pus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1; // is variable lenght: yes 

    memset(&attr_char_value, 0, sizeof(attr_char_value));


     ble_pus_meas_t initial_meas;
     memset(&initial_meas, 0, sizeof(initial_meas));
    uint8_t               init_value[20];

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_offs = 0;
 
    
    attr_char_value.max_len   = 20;
    attr_char_value.init_len  = ble_measurement_encode(&initial_meas,init_value);
    attr_char_value.p_value  =init_value;

    //attr_char_value.p_value  = &initial_custom_value;
    //attr_char_value.init_len  = sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(p_pus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_pus->dt_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);
//------------------------------------------------------------------------------------------------------------------------------
    // Add Custom Value characteristic integ
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    // the cccd_md struture is to enable notification or indication
    cccd_md.write_perm = p_pus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_pus->uuid_type;
    ble_uuid.uuid = ACCEL_INTEG_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_pus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_pus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    memset(&initial_meas, 0, sizeof(initial_meas));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_offs = 0;

    attr_char_value.max_len   = 20;
    attr_char_value.init_len  = ble_measurement_encode(&initial_meas,init_value);
    attr_char_value.p_value  = init_value;
    

    err_code = sd_ble_gatts_characteristic_add(p_pus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_pus->integ_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);
//------------------------------------------------------------------------------------------------------------------------------
    // Add Custom Value characteristic peak
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_pus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_pus->uuid_type;
    ble_uuid.uuid = ACCEL_PEAK_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_pus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_pus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));


    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_offs = 0;


    attr_char_value.max_len   = 20;
    attr_char_value.init_len  = ble_measurement_encode(&initial_meas,init_value);
    attr_char_value.p_value  = init_value;
    

    err_code = sd_ble_gatts_characteristic_add(p_pus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_pus->peak_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);



    return NRF_SUCCESS;
}

//------------------------------------------------------------------------------------------------------------------------------
//--------------------------  init service  ------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------

/**@brief initialize the Bluetooth table; adding service and charakteristic
 *
 * @param[in]   p_pus        peripeheral unified structure.
 * @param[in]   p_pus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_pus_init(ble_pus_t * p_pus, const ble_pus_init_t * p_pus_init)
{
    if (p_pus == NULL || p_pus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_pus->evt_handler               = p_pus_init->evt_handler;
    p_pus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = CUS_BASE_UUID;
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_pus->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_pus->uuid_type;
    ble_uuid.uuid = BLE_UUID_PUS_SERVICE;
    
    //BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEART_RATE_SERVICE);

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_pus->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Custom Value characteristic
    return custom_value_char_add(p_pus, p_pus_init);
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
// Update functions for char
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

/**@brief writing to the bluetooth table to update the characteristic dt 
 *
 * @param[in]   p_pus        peripeheral unified structure.
 * @param[in]   dt           value to save and send over Bluetooth
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */


uint32_t ble_pus_sample_time_value_update(ble_pus_t * p_pus, float dt)
{

    if (p_pus == NULL)
    {
        return NRF_ERROR_NULL;
    }

#if DEBUG_LOG
    MY_LOG( "\nsampling time:  " NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(dt));
#endif
    NRF_LOG_DEBUG("In ble_cus_custom_value_update. \r\n"); 
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    ble_pus_meas_t meas_data;
    meas_data.dt_value_in_float.exponent = 5;
    meas_data.dt_value_in_float.mantissa = (int)(dt*100000);

   
    if ((p_pus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;
        uint8_t                encoded_hts_meas[5] = {0};
        uint16_t               len;
        uint16_t               hvx_len;
        memset(&hvx_params, 0, sizeof(hvx_params));

        len     = ble_measurement_encode(&meas_data,encoded_hts_meas);;
        hvx_len = len;

        hvx_params.handle = p_pus->dt_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hts_meas;
#if DEBUG_LOG
        MY_LOG( "\t sampling time raw: \t %x %x %x %x,\r\n", encoded_hts_meas[0],  encoded_hts_meas[1], encoded_hts_meas[2],  encoded_hts_meas[3]); 
        //MY_LOG( "conn handle: %d,\r\n",p_cus->conn_handle); 
#endif
        err_code = sd_ble_gatts_hvx(p_pus->conn_handle, &hvx_params);
        NRF_LOG_DEBUG("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
            NRF_LOG_DEBUG("sd_ble_gatts_hvx result: NRF_ERROR_DATA_SIZE \r\n"); 
        }
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_DEBUG("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }

 
    return err_code;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

/**@brief writing to the bluetooth table to update the characteristic integ 
 *
 * @param[in]   p_pus        peripeheral unified structure.
 * @param[in]   integ           value to save and send over Bluetooth
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_pus_integ_value_update(ble_pus_t * p_pus, float integ)
{

    NRF_LOG_DEBUG("In ble_cus_custom_value_update. \r\n"); 
    if (p_pus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
    ble_pus_meas_t meas_data;
    meas_data.dt_value_in_float.exponent = 3;
    meas_data.dt_value_in_float.mantissa = (int)(integ*1000);

  
    #if DEBUG_LOG
    MY_LOG( "integ: \t\t" NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(integ));
    #endif

    // Send value if connected and notifying.
    if ((p_pus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;
        uint8_t                encoded_hts_meas[20] = {0};
        uint16_t               len;
        uint16_t               hvx_len;
        memset(&hvx_params, 0, sizeof(hvx_params));
        len     = ble_measurement_encode(&meas_data,encoded_hts_meas);;
        hvx_len = len;

        hvx_params.handle = p_pus->integ_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;


        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hts_meas;
#if DEBUG_LOG
        MY_LOG( "\t integ raw: \t\t %x %x %x %x,\r\n", encoded_hts_meas[0],  encoded_hts_meas[1], encoded_hts_meas[2],  encoded_hts_meas[3]);
#endif
        err_code = sd_ble_gatts_hvx(p_pus->conn_handle, &hvx_params);
        NRF_LOG_DEBUG("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_DEBUG("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }

    return err_code;
}

/**@brief writing to the bluetooth table to update the characteristic peak 
 *
 * @param[in]   p_pus        peripeheral unified structure.
 * @param[in]   peak           value to save and send over Bluetooth
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_pus_peak_value_update(ble_pus_t * p_pus, float peak)
{
#if DEBUG_LOG    
    MY_LOG( "peak: \t\t" NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(peak));
#endif
    
    if (p_pus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
    ble_pus_meas_t meas_data;
    meas_data.dt_value_in_float.exponent = 3;
    meas_data.dt_value_in_float.mantissa = (int)(peak*1000);



    // Send value if connected and notifying.
    if ((p_pus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;
        uint8_t                encoded_hts_meas[20] = {0};
        uint16_t               len;
        uint16_t               hvx_len;
        memset(&hvx_params, 0, sizeof(hvx_params));
        len     = ble_measurement_encode(&meas_data,encoded_hts_meas);
        hvx_len = len;

        hvx_params.handle = p_pus->peak_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hts_meas;
#if DEBUG_LOG
        MY_LOG( "\t peak raw: \t\t %x %x %x %x,\r\n", encoded_hts_meas[0],  encoded_hts_meas[1], encoded_hts_meas[2],  encoded_hts_meas[3]);
#endif
        err_code = sd_ble_gatts_hvx(p_pus->conn_handle, &hvx_params);
        NRF_LOG_DEBUG("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_DEBUG("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }

    return err_code;
}


/**@brief The function encodes the buffer with the raw bytes values from p_pus_meas: 
 *        It takes the exponent and the mantis values. 
 *       
 * @param[in]   p_pus_meas            measurment structure with mantis and exponent
 * @param[in]   p_encoded_buffer      buffer where the data needs to be send
 *
 * @return      len                   number of bytes it has been written
 */

static uint8_t ble_measurement_encode(ble_pus_meas_t* p_pus_meas, uint8_t* p_encoded_buffer)
{
    uint8_t  len   = 0;
    uint32_t encoded_temp;

    encoded_temp = ((p_pus_meas->dt_value_in_float.exponent << 24) & 0xFF000000) |
                   ((p_pus_meas->dt_value_in_float.mantissa <<  0) & 0x00FFFFFF);

    p_encoded_buffer[0] = (uint8_t) ((encoded_temp & 0x000000FF) >> 0);
    p_encoded_buffer[1] = (uint8_t) ((encoded_temp & 0x0000FF00) >> 8);
    p_encoded_buffer[2] = (uint8_t) ((encoded_temp & 0x00FF0000) >> 16);
    p_encoded_buffer[3] = (uint8_t) ((encoded_temp & 0xFF000000) >> 24);
    len += sizeof(uint32_t);

    return len;

}