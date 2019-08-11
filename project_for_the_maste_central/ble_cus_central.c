#include "sdk_common.h"
#include "ble_cus_central.h"
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

/**@brief Function for discovering the device and reading out the cccd and value handles
 *
 * @param[in]   p_ble_cus_c       Custom Service structure to store the data
 * @param[in]   p_ble_evt         Event received from the BLE stack.
 */
void ble_cus_c_on_db_disc_evt(ble_cus_t* p_ble_cus_c, ble_db_discovery_evt_t* p_evt)
{

    // Check if the peripeheral was discovered with the right uuid and service number
    if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
        &&  (p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_CUS_SERVICE)
        &&  (p_evt->params.discovered_db.srv_uuid.type == p_ble_cus_c->uuid_type))
    { 

      ble_cus_evt_t cus_c_evt;
      memset(&cus_c_evt,0,sizeof(ble_cus_evt_t));
      ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

      // iterate over the all charakteristic that we got and save the important info; cccd  and value handle
      for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
      {
            //MY_LOG(" XX ble_cus_c_on_db_disc_evt: iter %d, uuid: 0x%x \n",i, p_chars[i].characteristic.uuid.uuid);
            switch (p_chars[i].characteristic.uuid.uuid)
            {
                case ACCEL_DT_CHAR_UUID:
                {
                    cus_c_evt.params.cus_db.sampling_time_cccd_handle = 
                            p_evt->params.discovered_db.charateristics[i].cccd_handle;
                    cus_c_evt.params.cus_db.sampling_time_handle = 
                            p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
               //     MY_LOG(" ACCEL_DT_CHAR_UUID: cccd 0x%x  and value_handle 0x%x \n", 
               //             cus_c_evt.params.cus_db.integ_cccd_handle, cus_c_evt.params.cus_db.integ_handle);
                }
                    break;

                case ACCEL_INTEG_CHAR_UUID:
                {
                    cus_c_evt.params.cus_db.integ_cccd_handle = 
                            p_evt->params.discovered_db.charateristics[i].cccd_handle;
                    cus_c_evt.params.cus_db.integ_handle = 
                            p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                //    MY_LOG("ACCEL_INTEG_CHAR_UUID: cccd 0x%x  and value_handle 0x%x \n", 
                //            cus_c_evt.params.cus_db.integ_cccd_handle, cus_c_evt.params.cus_db.integ_handle);
                }
                    break;

                case ACCEL_PEAK_CHAR_UUID:
                {
                    cus_c_evt.params.cus_db.peak_cccd_handle = 
                            p_evt->params.discovered_db.charateristics[i].cccd_handle;
                    cus_c_evt.params.cus_db.peak_handle = 
                            p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                   // MY_LOG(" ACCEL_PEAK_CHAR_UUID: cccd 0x%x  and value_handle 0x%x \n", 
                   //         cus_c_evt.params.cus_db.integ_cccd_handle, cus_c_evt.params.cus_db.integ_handle);
                   }   
                    break;

                case CUSTOM_VALUE_CHAR_UUID:  
                    //NOTE: not implemented, because there is no needed for it
                                        //MY_LOG("ble_cus_c_on_db_disc_evt: not implemented CUSTOM_VALUE_CHAR_UUID\n");
                    break;


                default:
                    MY_LOG("__________XXX iter on %d "
                           "not defined charakteristic: 0x%x\n",
                            i,
                            p_chars[i].characteristic.uuid.uuid);
                    break;
            }
      }
        
      // check if event handler was provided by the initialization
      // if so propagate the event
      if (p_ble_cus_c->evt_handler != NULL)
      {
        //MY_LOG("ble_cus_c_on_db_disc_evt: BLE_CUS_EVT_DISCOVERY_COMPLETE\n");
        cus_c_evt.conn_handle = p_evt->conn_handle;
        cus_c_evt.evt_type    = BLE_CUS_EVT_DISCOVERY_COMPLETE;
        p_ble_cus_c->evt_handler(p_ble_cus_c, &cus_c_evt);
      }
    }
    
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
/*
static void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    ble_cus_evt_t evt;
    evt.evt_type = BLE_CUS_EVT_CONNECTED;

}
*/

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    
    if (p_cus->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_cus->conn_handle                      = BLE_CONN_HANDLE_INVALID;
        p_cus->cus_db.integ_cccd_handle         = BLE_GATT_HANDLE_INVALID;
        p_cus->cus_db.integ_handle              = BLE_GATT_HANDLE_INVALID;

        p_cus->cus_db.peak_cccd_handle          = BLE_GATT_HANDLE_INVALID;
        p_cus->cus_db.peak_handle               = BLE_GATT_HANDLE_INVALID;

        p_cus->cus_db.sampling_time_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_cus->cus_db.sampling_time_handle      = BLE_GATT_HANDLE_INVALID;
    }
    
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
    ble_cus_evt_t evt;
    evt.evt_type = BLE_CUS_EVT_DISCONNECTED;
    p_cus->evt_handler(p_cus, &evt);
}

static ble_cus_evt_t ble_cus_c_evt_m; // statical data types for storing all the data from BLE sending


/**@brief Function for handling the notification events 
 *
 * @param[in]   p_cus       Central unified structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
 
static void on_hvx(ble_cus_t * p_ble_cus_c, ble_evt_t const * p_ble_evt)
{
  
    // HVX can only occur from client sending.
    if (p_ble_cus_c->cus_db.integ_cccd_handle != BLE_GATT_HANDLE_INVALID)
    {

        if(p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_cus_c->cus_db.integ_handle){
          
            ble_cus_c_evt_m.params.cus.integration_value[0] = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            ble_cus_c_evt_m.params.cus.integration_value[1] = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            ble_cus_c_evt_m.params.cus.integration_value[2] = p_ble_evt->evt.gattc_evt.params.hvx.data[2];
            ble_cus_c_evt_m.params.cus.integration_value[3] = p_ble_evt->evt.gattc_evt.params.hvx.data[3];
            ble_cus_c_evt_m.params.cus.integration_value_len = p_ble_evt->evt.gattc_evt.params.hvx.len;
        }
        else if(p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_cus_c->cus_db.sampling_time_handle){

            ble_cus_c_evt_m.params.cus.sampling_time[0] = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            ble_cus_c_evt_m.params.cus.sampling_time[1] = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            ble_cus_c_evt_m.params.cus.sampling_time[2] = p_ble_evt->evt.gattc_evt.params.hvx.data[2];
            ble_cus_c_evt_m.params.cus.sampling_time[3] = p_ble_evt->evt.gattc_evt.params.hvx.data[3];
            ble_cus_c_evt_m.params.cus.sampling_time_value_len = p_ble_evt->evt.gattc_evt.params.hvx.len;

        }
        else if(p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_cus_c->cus_db.peak_handle){

            ble_cus_c_evt_m.params.cus.peak[0] = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            ble_cus_c_evt_m.params.cus.peak[1] = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            ble_cus_c_evt_m.params.cus.peak[2] = p_ble_evt->evt.gattc_evt.params.hvx.data[2];
            ble_cus_c_evt_m.params.cus.peak[3] = p_ble_evt->evt.gattc_evt.params.hvx.data[3];
            ble_cus_c_evt_m.params.cus.peak_value_len = p_ble_evt->evt.gattc_evt.params.hvx.len;

            if(p_ble_cus_c->evt_handler != NULL)
            {
                ble_cus_c_evt_m.evt_type = BLE_CUS_C_EVT_NUS_TX_EVT;
                //MY_LOG("Client sending data.");
                p_ble_cus_c->evt_handler(p_ble_cus_c, &ble_cus_c_evt_m);
                
            }
        }
        else
        {
            MY_LOG("on_hvx: not implemented CUSTOM_VALUE_CHAR_UUID\n");
            MY_LOG("on_hvx: error with disc on db,"
           "not defined charakteristic: 0x%x\n",
            1);
        }
    }
}


/**@brief Function for handling the BLE cus event.
 *
 * @param[in]   p_context       Custom Service structure.
 * @param[in]   p_ble_evt       Event received from the BLE stack.
 */

void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{

    ble_cus_t * p_cus_c = (ble_cus_t *) p_context;  
    if (p_cus_c == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {

        case BLE_GAP_EVT_CONNECTED: // connection event was handled
            p_cus_c->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            //on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE: // the update event was issued, the central is now ready
            ///MY_LOG("ble_cus_on_ble_evt: BLE_GAP_EVT_CONN_PARAM_UPDATE: code %x\n",p_ble_evt->header.evt_id); 
            MY_LOG("The central master is ready!!!\n"); 
            break;



        case BLE_GAP_EVT_DISCONNECTED: // an disconection event was issued
            MY_LOG("ble_cus_on_ble_evt: BLE_GAP_EVT_DISCONNECTED: code %x\n",p_ble_evt->header.evt_id); 
            on_disconnect(p_cus_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            //MY_LOG("ble_cus_on_ble_evt: BLE_GATTC_EVT_WRITE_RSP: code \n");//%x\n",p_ble_evt->header.evt_id); 
            //on_write(p_cus, p_ble_evt);
            break;

        case BLE_GATTC_EVT_HVX: // notification even from the peripeheral 
            //MY_LOG("ble_cus_on_ble_evt: BLE_GATTC_EVT_HVX: code %x\n",p_ble_evt->header.evt_id);
            on_hvx(p_cus_c, p_ble_evt); 
            break;

        default:
            // the default case is only valuable if the triggered events want to be figured out, which  may not be implemented
            //MY_LOG("ble_cus_on_ble_evt: Event number = %x\r\n", p_ble_evt->header.evt_id); 
            break;
    }
}

/**@brief Assign the data and connecdtions handles to the p_ble_cus structure 
 *
 * @param[in]   p_ble_cus       Custom Service structure.
 * @param[in]   conn_handle     connection handle number between the peripeheral and central
 * @param[in]   p_db_handles    handles where the data and cccd is
 */
uint32_t ble_cus_c_handles_assign(ble_cus_t*                  p_ble_cus,
                                  uint16_t                    conn_handle,
                                  const ble_cus_c_db_t*    p_db_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_cus);
    p_ble_cus->conn_handle = conn_handle;
    if (p_db_handles != NULL)
    {
        p_ble_cus->cus_db = *p_db_handles;

    }

    return NRF_SUCCESS;
}

/**@brief Enable notification for the characteristic 
 *
 * @param[in]   conn_handle     connection handle for the BLE communication
 * @param[in]   cccd_handle     the handle number to the charachteristic
 * @param[in]   enable          enabling and dissabling the notification
 */

uint32_t ble_cus_notif_enable(uint16_t conn_handle,  uint16_t cccd_handle, bool enable)
{
     
    if ( (conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    //return cccd_configure(conn_handle,p_ble_nus_c->handles.nus_tx_cccd_handle, true);
    uint8_t buf[BLE_CCCD_VALUE_LEN];

    buf[0] = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    buf[1] = 0;

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = cccd_handle,
        .offset   = 0,
        .len      = sizeof(buf),
        .p_value  = buf
    };
    return sd_ble_gattc_write(conn_handle, &write_params);

}


/**@brief Initialize and register the ble cus to the sdk stack
 *
 * @param[in]   p_cus              pointer to the cus structure
 * @param[in]   ble_cus_init_t     struct for init the cus
 */
uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    VERIFY_PARAM_NOT_NULL(p_cus);
    VERIFY_PARAM_NOT_NULL(p_cus_init);

    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    //ble_uuid128_t base_uuid = {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x8E}};
    ble_uuid128_t base_uuid = CUS_BASE_UUID;

    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = BLE_UUID_CUS_SERVICE;

    // Initialize service structure
    p_cus->evt_handler                      = p_cus_init->evt_handler;
    p_cus->conn_handle                      = BLE_CONN_HANDLE_INVALID;
    p_cus->cus_db.integ_cccd_handle         = BLE_GATT_HANDLE_INVALID ;
    p_cus->cus_db.integ_handle              = BLE_GATT_HANDLE_INVALID ;
    p_cus->cus_db.peak_cccd_handle          = BLE_GATT_HANDLE_INVALID ;
    p_cus->cus_db.peak_handle               = BLE_GATT_HANDLE_INVALID ;
    p_cus->cus_db.sampling_time_cccd_handle = BLE_GATT_HANDLE_INVALID ;
    p_cus->cus_db.sampling_time_handle      = BLE_GATT_HANDLE_INVALID ;
 
    // Add Custom Value characteristic
    return ble_db_discovery_evt_register(&ble_uuid);
}
