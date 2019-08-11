#include "steval_MKI1178V1_central.h"
#include "my_log.h"



#define MY_LOG(...) \
do { \
 	char str[64];\
 	sprintf(str, __VA_ARGS__);\
 	SEGGER_RTT_WriteString(0, str);\
 } while(0)


#define SIZE 2000
static int size_of_data = -1; // -1 indicates no data in the array
static Accel_data_p_t data[SIZE] = {0}; // structure for saving the data points of the acceleraation

Accel_data_p_t point = {0};  // for debugging porpuses 


// saving the internal register in internal values 
uint8_t event_detection_status  = 0;                // axis event status: gives the information which 3 axis have been triggered  
uint8_t CTRL1_value             = 0;                // intern status register for handling the intern events of the accelerometer 
uint8_t FIFO_status             = 0;                // intern status register for handling the intern events of the accelerometer
uint8_t status                  = 0;                // intern status register for handling the intern events of the accelerometer



void MKI1178V1_save_data()
{
   ret_code_t err_code;
   err_code = i2c_read2(FIFO_SAMPLES,&FIFO_status);
   APP_ERROR_CHECK(err_code);
   uint8_t FIFO_samples = FIFO_status & FIFO_CTRL_MASK; // mask to only get the number of FIFO samples

     for (; FIFO_samples ; --FIFO_samples)
    {
      size_of_data++;
      Read_X_Data(&(data[size_of_data].values[X_POSITION]));
      Read_Y_Data(&(data[size_of_data].values[Y_POSITION]));
      Read_Z_Data(&(data[size_of_data].values[Z_POSITION]));
      
    }

}

static void start_calculation(void)
{
  // 
  uint8_t get_ODR_and_MODE_values = CTRL1_value >> 2; // the first nibble we are not interested in

  // get sampling time of the accelerometer:
  // dt_values[get the ODR][get the MODE] : ODR: output data rate, MODE: mode selection
  dt = dt_values[(get_ODR_and_MODE_values>>2)][get_ODR_and_MODE_values & 0x03]; 

  int triggered_axis = (event_detection_status & 0x07); // only the first 3 bits are important to us
	
  // find out which axis was first triggered
  if( (triggered_axis & Y_POSITION) == Y_POSITION)
          triggered_axis = 0;
  else if ( (triggered_axis & X_POSITION) == X_POSITION)
          triggered_axis = 1;
  else
          triggered_axis = 2;
  
  // initalize the extreme values and integration value to 0
  max_value = data[0].values[triggered_axis];
  min_value = data[0].values[triggered_axis];
  integ_value = 0;
	
  int sign_of_data = (int)data[1].values[triggered_axis] & 0x80000000; // get the sing of the value
  
  // integration rule: trapez method (the core functionality)
  for (iter = 1; iter < size_of_data+1;++iter)
  {
      // check if the sign of the data has changed. If so, then the pulse is already finished and finish the calculation 
      if ( ( ( (int) data[iter].values[triggered_axis]) & 0x80000000) ^ sign_of_data ) 
              break;
    
      // find the extreme values
      if(data[iter].values[triggered_axis] >  max_value)
      {
          max_value = data[iter].values[triggered_axis];
      }

      if(data[iter].values[triggered_axis] < min_value)
      {
          min_value = data[iter].values[triggered_axis];
      }
    
      // calculate the surface of the integration and add it to the integ value
      differ = data[iter].values[triggered_axis] - data[iter - 1].values[triggered_axis];  
      integ_value += ( data[iter-1].values[triggered_axis] + (differ/2) )*dt;
  }
	
  sampling_time = (dt* iter);
   
  if (integ_value > 0)
  {
          peak = max_value;
  } 
  else
  {
          peak = min_value;
  }
}

static void send_data(void){
    char dataMsg[64] = {0};
    size_t i = iter +EXTRA_VALUES_TO_SEND;
    int threshold_value = TRESHOLD_VALUE_MG;

    sprintf(dataMsg,"start %i %i " NRF_LOG_FLOAT_MARKER" %i", i, iter, NRF_LOG_FLOAT(dt), threshold_value);
    NRF_LOG_RAW_INFO("%s \n", dataMsg);
    MY_LOG("\n%s \n", dataMsg);
    //NRF_LOG_RAW_INFO("\n%s \n", dataMsg);
    NRF_LOG_FLUSH();
    for(int idx = 0 ;idx < i; idx++)
    { 

      sprintf(dataMsg,
              NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER,
              NRF_LOG_FLOAT(data[idx].values[X_POSITION]),
              NRF_LOG_FLOAT(data[idx].values[Y_POSITION]),
              NRF_LOG_FLOAT(data[idx].values[Z_POSITION]));

      NRF_LOG_RAW_INFO( "%s \n", dataMsg);     
      MY_LOG( "%s \n", dataMsg);     
    }
     MY_LOG("-------------------------------------------------------\n");
    //NRF_LOG_FLUSH();

}

static void stop_sampling(void)
{
    ret_code_t err_code  = i2c_write2(CTRL1, CTRL1_STOP_THE_SENSOR); // stop the sensor
    APP_ERROR_CHECK(err_code);

    err_code  = i2c_write2(FIFO_CTRL,FIFO_CTRL_STOP); // stop the sensor
    APP_ERROR_CHECK(err_code);
    MKI1178V1_save_data();

}

void put_in_active_mode(void)
{
    ret_code_t err_code;
    /* prepare for the next data package */
    size_of_data = -1;

    // start again --> configure the FIFO and the control register 1
    err_code = i2c_write2(FIFO_CTRL, FIFO_CTRL_START); // configure the bypas-to-continues mode | set threshold to 20 
    APP_ERROR_CHECK(err_code);

    err_code = i2c_write2(CTRL1, CTRL1_value); // FIFO threshold interrupt is routed to INT2
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */

accel_t processing()
{

        stop_sampling();
        start_calculation();   
#if UART_SEND_DATA
        send_data();
#endif
        accel_t data_main;
        data_main.integ_value=integ_value;
        data_main.peak=peak;
        data_main.sampling_time=sampling_time;
  
        if(size_of_data > -1){
          memset(&data, 0, size_of_data*sizeof(Accel_data_p_t));
          return data_main;
        }

}

void MKI1178V1_get_triggered_axis()
{
   ret_code_t err_code = i2c_read2(WAKE_UP_SRC,&event_detection_status);
   APP_ERROR_CHECK(err_code);
}


static void notification_timeout_handler(void * p_context)
{

    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    APP_ERROR_CHECK(err_code);
    stop_sampling();
    start_calculation();    
#if UART_SEND_DATA
    send_data();
#endif
}

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
void twi_init2 (void)
{
    ret_code_t err_code = 0;

    NRF_TWIM0->PSEL.SCL = STEVAL_MKI1178V1_SCL_PIN;
    NRF_TWIM0->PSEL.SDA = STEVAL_MKI1178V1_SDA_PIN;

    NRF_TWIM0->ADDRESS = ADDRESS_MKI179_SAO_HIGH;
    NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400 << TWIM_FREQUENCY_FREQUENCY_Pos;
    NRF_TWIM0->SHORTS = 0;

    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
}

void MKI1178V1_int(){

    twi_init2();
    MKI1178V1_reset_accel();

    uint8_t data = {0};
    i2c_read2(WHO_AM_I,&data);
    if(WHO_AM_I_ANSWER != data)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
    }

}

//void MKI1178V1_conf(accel_event evt_struct)
void MKI1178V1_conf(void)
{
  //evt_handler_aljosa = evt_struct.evt_handler_struct;
  MKI1178V1_int();
  MKI1178V1_conf_interrupt();
}

void MKI1178V1_conf_interrupt()
{
  ret_code_t err_code;

   err_code = empty_the_fifo_Buffer();
   APP_ERROR_CHECK(err_code);

  // enable interrupt lines 
  nrfx_gpiote_in_event_enable(STEVAL_MKI1178V1_INT1_PIN, true);
  nrfx_gpiote_in_event_enable(STEVAL_MKI1178V1_INT2_PIN, true);


  //err_code = i2c_write(CTRL6, CTRL6_FS_16g | CTRL6_ENABLE_LOW_NOISE_CONF);  // define the amplitued value and enable low nosei
  err_code = i2c_write2(CTRL6,AMPLITUDE_RANGE_WITH_LOW_NOISE);  // define the amplitued value and enable low nosei
  APP_ERROR_CHECK(err_code);
  //i2c_read(CTRL6,&data);


  err_code = i2c_write2(CTRL7, CTRL7_INTERRUTPS_ENABLE | CTRL7_USR_OFF_W | CTRL7_USR_OFF_ON_OUT);  // interrupt enable | add a user offset weight | enable the output with the offset
  APP_ERROR_CHECK(err_code);
  //i2c_read(CTRL7,&data);


  err_code = i2c_write2(X_OFS_USR, X_OFS_USR_VALUE); // adding -1 to the OFFSET_X_REG
  APP_ERROR_CHECK(err_code);
  //i2c_read(X_OFS_USR,&data);


  err_code = i2c_write2(Y_OFS_USR, Y_OFS_USR_VALUE); // adding -2 to the OFFSET_Y_REG
  APP_ERROR_CHECK(err_code);
  //i2c_read(Y_OFS_USR,&data);

  err_code = i2c_write2(Z_OFS_USR, Z_OFS_USR_VALUE); // adding -1g to the OFFSET_Z_REG
  APP_ERROR_CHECK(err_code);
  //i2c_read(Z_OFS_USR,&data);


  err_code = i2c_write2(WAKE_UP_DUR, WAKE_UP_DUR_VALUE); // set up wake up duration to zero
  APP_ERROR_CHECK(err_code);
  //i2c_read(WAKE_UP_DUR,&data);



  uint8_t bit_presentation_for_the_treshold=
        transfor_threshold_value_for_the_register(AMPLITUDE_RANGE_WITH_LOW_NOISE,TRESHOLD_VALUE_MG);

  err_code = i2c_write2(WAKE_UP_THS, bit_presentation_for_the_treshold); // set up wake up threshold value
  APP_ERROR_CHECK(err_code);
  //i2c_read(WAKE_UP_THS,&data);

  err_code = i2c_write2(CTRL4_INT_PAD_CRL,CTRL4_INT_PAD_CRL_ROUTE_INT1_TO_WAKE_UP); // Wake-up recongnition is routed to INT1
  APP_ERROR_CHECK(err_code);
  //i2c_read(CTRL4_INT_PAD_CRL,&data);


  err_code = i2c_write2(CTRL5_INT_PAD_CRL, CTRL5_INT_PAD_CRL_VALUE_FIFO_THRESHOLD); // FIFO threshold interrupt is routed to INT2
  APP_ERROR_CHECK(err_code);
  //i2c_read(CTRL5_INT_PAD_CRL,&data);


  err_code = i2c_write2(FIFO_CTRL, FIFO_CTRL_START); // configure the bypas-to-continues mode | set threshold to 20 
  APP_ERROR_CHECK(err_code);
  //i2c_read(FIFO_CTRL,&data);


  //CTRL1_value = CTRL1_ODR_50Hz; 
  CTRL1_value = CTRL1_ODR_HIGH_PEROFRAMNCE_800Hz; 
  err_code = i2c_write2(CTRL1, CTRL1_value); // FIFO threshold interrupt is routed to INT2
  APP_ERROR_CHECK(err_code);
  //i2c_read(CTRL1, &data);

}

static void MKI1178V1_reset_accel(){
    i2c_write2(CTRL2, CTRL2_SOFT_RESET);   
    nrf_delay_ms(1);
    i2c_write2(CTRL2, CTRL2_BOOT);   
    nrf_delay_ms(20);

}


ret_code_t empty_the_fifo_Buffer()
{
  ret_code_t err_code;
  err_code = i2c_read2(STATUS,&status);
  APP_ERROR_CHECK(err_code);

  while ( (status & STATUS_DRDY) == STATUS_DRDY){ //look if data is ready
      // the arrangement of the data values x,y,z are tighted to the register WAKE_UP_SRC
      Read_X_Data(&(point.values[X_POSITION]));  
      Read_Y_Data(&(point.values[Y_POSITION]));
      Read_Z_Data(&(point.values[Z_POSITION]));
      err_code = i2c_read2(STATUS,&status);
      APP_ERROR_CHECK(err_code);
  }

}

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */

static void Read_X_Data(float* Value){

    uint8_t temp[2] = {0,0};
    int16_t tmp16 = 0;

    ret_code_t err_code  = i2c_read2(OUT_X_L, &temp[0]);
    APP_ERROR_CHECK(err_code);
    
    err_code  = i2c_read2(OUT_X_H, &temp[1]); 
    APP_ERROR_CHECK(err_code);
    tmp16 = (int16_t) (temp[1] << 8) | (int16_t)temp[0];	

    int index = (AMPLITUDE_RANGE_WITH_LOW_NOISE >> 4 ) & 0x03; // the first 2 bits of the second Nibble is what we want

    // I do this because there is a not linear factorbetween 8g and 16g
#if AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_16g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/4) * calculation_values[index][1];
#elif AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_8g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/2) * calculation_values[index][1];
#else
    *Value = (double)(tmp16) / 	((calculation_values[index][0])) * calculation_values[index][1];
#endif

}

static void Read_Y_Data(float* Value){

    uint8_t temp[2] = {0,0};
    int16_t tmp16 = 0;

    ret_code_t err_code  = i2c_read2(OUT_Y_L, &temp[0]);
    APP_ERROR_CHECK(err_code);


    err_code  = i2c_read2(OUT_Y_H, &temp[1]); 
    APP_ERROR_CHECK(err_code);
    tmp16 = (int16_t) (temp[1] << 8) | (int16_t)temp[0];	

    int index = (AMPLITUDE_RANGE_WITH_LOW_NOISE >> 4 ) & 0x03; // the first 2 bits of the second Nibble is what we want

    // I do this because there is a not linearfactorbetween 8g and 16g
#if AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_16g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/4) * calculation_values[index][1];
#elif AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_8g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/2) * calculation_values[index][1];
#else
    *Value = (double)(tmp16) / 	((calculation_values[index][0])) * calculation_values[index][1];
#endif

}

static void Read_Z_Data(float* Value){

    uint8_t temp[2] = {0,0};
    int16_t tmp16 = 0;


    ret_code_t err_code  = i2c_read2(OUT_Z_L, &temp[0]);
    APP_ERROR_CHECK(err_code);

    err_code  = i2c_read2(OUT_Z_H, &temp[1]); 
    APP_ERROR_CHECK(err_code);
    tmp16 = (int16_t) (temp[1] << 8) | (int16_t)temp[0];	

    // the first 2 bits of the second Nibble is what we want
    int index = (AMPLITUDE_RANGE_WITH_LOW_NOISE >> 4 ) & 0x03; 

    // I do this because there is a not linearfactorbetween 8g and 16g
#if	AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_16g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/4) * calculation_values[index][1];
#elif AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_8g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/2) * calculation_values[index][1];
#else
    *Value = (double)(tmp16) / 	((calculation_values[index][0])) * calculation_values[index][1];
#endif

}

static ret_code_t i2c_write2(uint8_t addr, uint8_t data)
{
  uint8_t tx_buf[2];
  NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STOP_Msk;

  tx_buf[0] = addr;
  tx_buf[1] = data;
  NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
  NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf[0];

  NRF_TWIM0->EVENTS_STOPPED = 0;
  NRF_TWIM0->TASKS_STARTTX = 1;
  while (NRF_TWIM0->EVENTS_STOPPED == 0);
  return NRF_SUCCESS;

}

/**@brief Function for reading the i2c messages
 *
 * @return     NRF_SUCCESS when we received a value.
 */
static ret_code_t i2c_read2(uint8_t addr, uint8_t* data_intern)
{
  uint8_t tx_buf[1];
  uint8_t data_temp = *data_intern;
  NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STARTRX_Msk | TWIM_SHORTS_LASTRX_STOP_Msk;

  tx_buf[0] = addr;
  NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
  NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf[0];

  NRF_TWIM0->RXD.MAXCNT = 1;//sizeof(data);

  NRF_TWIM0->RXD.PTR = (uint32_t)data_intern;

  NRF_TWIM0->EVENTS_STOPPED = 0;
  NRF_TWIM0->TASKS_STARTTX = 1;
  while (NRF_TWIM0->EVENTS_STOPPED == 0);

  if(NRF_TWIM0->ERRORSRC != NRF_SUCCESS)
      return NRF_ERROR_INTERNAL;

  return NRF_SUCCESS;
}
/**@brief Function for converting the force range from mg to the register value 
 *
 * @return      returns the range in in byte corresponding to the FR of the register 
 */

static uint8_t transfor_threshold_value_for_the_register(int range_in_g,int value_in_mg)
{
  // we are only interested in the last nibble, because there are the range configuration stored
  int bit_value_for_force_range =  range_in_g & 0xF0;
  int Register_Value = 0;

  /*
    equation from datasheet:
    'Register_Value' * 'Force_Range' / 128 = 'Threshold_value_in_g' 
    Example:
     Register_Value=48
     Force_Range=2g;
    ______________________________________________________________
     Result = 0,75 g = 750 mg
    we want opposite--> so we reverse the formula and get:
    'Threshold_value_in_g' * 128 / 'Force_Range' = 'Register_Value'
    ================================================================


  */

  switch(bit_value_for_force_range){
    case CTRL6_FS_2g:
      Register_Value = value_in_mg<<6 / CONVERT_G_TO_MG; // mutliplication by 64 and divide by 1000 to convert to mg
      break;
    case CTRL6_FS_4g:
      Register_Value = value_in_mg<<5 / CONVERT_G_TO_MG; // mutliplication by 32 and divide by 1000 to convert to mg
      break;
    case CTRL6_FS_8g:
      Register_Value = ((value_in_mg*64)+500) / CONVERT_G_TO_MG; // mutliplication by 16 and divide by 1000 to convert to mg
      //Register_Value = ceil((value_in_mg<<2) / CONVERT_G_TO_MG); // mutliplication by 16 and divide by 1000 to convert to mg
#if DEBUG_LOG
#if MY_RTT    
      MY_LOG("Threshold value: %d\n", Register_Value);
#else
    NRF_LOG_RAW_INFO("Threshold value: %d\n", Register_Value);
#endif
#endif

      break;
    case CTRL6_FS_16g:
      Register_Value = value_in_mg<<3 / CONVERT_G_TO_MG; // mutliplication by 8 and divide by 1000 to convert to mg
      break;
  }

    return (uint8_t)Register_Value;
}