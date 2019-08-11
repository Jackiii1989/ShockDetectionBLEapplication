#include "steval_MKI1178V1.h"
#include "my_log.h"

#define SIZE 2000

#define MY_LOG(...) \
do { \
 	char str[64];\
 	sprintf(str, __VA_ARGS__);\
 	SEGGER_RTT_WriteString(0, str);\
 } while(0)



static int size_of_data = -1; // -1 indicates no data in the array
static Accel_data_p_t data[SIZE] = {0}; // structure for saving the data points of the acceleration

Accel_data_p_t point = {0};  // for debugging porpuses and for emptying the fifo buffer 

// saving the internal register in internal values 
uint8_t event_detection_status  = 0;                // axis event status: gives the information which 3 axis have been triggered  
uint8_t CTRL1_value             = 0;                // intern status register for handling the intern events of the accelerometer 
uint8_t FIFO_status             = 0;                // intern status register for handling the intern events of the accelerometer
uint8_t status                  = 0;                // intern status register for handling the intern events of the accelerometer

uint8_t trigger_value           = 0;                // intern status register for handling the intern events of the accelerometer
uint8_t quant_of_reg_val        = 0;                // intern status register for handling the intern events of the accelerometer




/**@brief after a fifo is full the save_data() function is called to empty the fifo 
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/

void MKI1178V1_save_data()
{
  ret_code_t err_code;

#if FIFO_ENABLE
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
#else

    do{
      size_of_data++;
      Read_X_Data(&(data[size_of_data].values[X_POSITION]));
      Read_Y_Data(&(data[size_of_data].values[Y_POSITION]));
      Read_Z_Data(&(data[size_of_data].values[Z_POSITION]));
      err_code = i2c_read2(STATUS_DUP,&status);
      APP_ERROR_CHECK(err_code);
    }
    while( (status &STATUS_DRDY) == STATUS_DRDY);

#endif
}

/**@brief calculates and extractes the parameters from the impact(duration of the impulse, peak an integration value)
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/
static void start_calculation(void)
{
  uint8_t get_ODR_and_MODE_values = CTRL1_value >> 2; // the first nibble we are not interested in

  // get sampling time of the accelerometer:
  // dt_values[get the ODR][get the MODE] : ODR: output data rate, MODE: mode selection
  dt = dt_values[(get_ODR_and_MODE_values>>2)][get_ODR_and_MODE_values & 0x03]; // get sampling time of the accelerometer

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
  
  // integration rule: trapez method 
  for (iter = 1; iter < size_of_data+1;++iter)
  {
      // check if the sign of the data changed. If so, then pulse already finished and finish the calculation 
      if ( ( ( (int) data[iter].values[triggered_axis]) & 0x80000000) ^ sign_of_data ) 
              break;
    
      // find extreme values
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


/**@brief sending the data over uart and rtt; first a string is send which defines the sampling paramters, 
 *        threshold an etc. The data is 
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/
static void send_data(void){
    char dataMsg[64] = {0};
    
    size_t i = iter +EXTRA_VALUES_TO_SEND;
    
    int threshold_value = trigger_value*250;
    
    sprintf(dataMsg,"start %i %i " NRF_LOG_FLOAT_MARKER" %i", i, iter, NRF_LOG_FLOAT(dt), threshold_value);
    //MY_LOG("\n%s \n", dataMsg);
    
    NRF_LOG_RAW_INFO("%s \n", dataMsg);
    MY_LOG( "%s \n", dataMsg); 
    for(int idx = 0 ;idx < i; idx++)
    { 

      sprintf(dataMsg,
              NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER" ",
              NRF_LOG_FLOAT(data[idx].values[X_POSITION]),
              NRF_LOG_FLOAT(data[idx].values[Y_POSITION]),
              NRF_LOG_FLOAT(data[idx].values[Z_POSITION]));

      NRF_LOG_RAW_INFO( "%s\n", dataMsg);     
      // for the rtt
      MY_LOG( "%s \n", dataMsg);     
    }
    // wait for 2s because matlab cannot handle the speed of sending the data; 
    // because some calculations is happening in matlab and withouht the delay it just skips the values
    nrf_delay_ms(2000);

}


/**@brief stops the sampling: turns the sensor into low power mode and turn offs the fifo 
 *         register
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/
static void stop_sampling(void)
{
    ret_code_t err_code  = i2c_write2(CTRL1, CTRL1_STOP_THE_SENSOR); // stop the sensor
    APP_ERROR_CHECK(err_code);

    err_code  = i2c_write2(FIFO_CTRL,FIFO_CTRL_STOP); // stop the fifo
    APP_ERROR_CHECK(err_code);
    MKI1178V1_save_data();

}


/**@brief puts it again in the active mode after the measurment was finished. 
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/
void put_in_active_mode(void)
{
    ret_code_t err_code;
    // prepare for the next data package
    size_of_data = -1;

    // start again --> configure the FIFO and the control register 1
    err_code = i2c_write2(FIFO_CTRL, FIFO_CTRL_START); // configure the bypas-to-continues mode | set threshold to 20 
    APP_ERROR_CHECK(err_code);

    err_code = i2c_write2(CTRL1, CTRL1_value); // FIFO threshold interrupt is routed to INT2
    APP_ERROR_CHECK(err_code);
}




/**@brief After the data is avaible the sensor is being put in lower power mode and the calculation is starting.
 *        The values are being calculated and extracted, and if enabled, the data is being send over UART/USB
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */



/**@brief starts the processing after the timeout of the timmer was reached. It sets the sensor in low power mode.
 *        The values are being calculated and extracted, and if enabled, the data is being send over UART/USB.
 *        
 *        
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      returns the calculated/ extracted values 
 *
**/

void processing(accel_t* data_main)
{

        stop_sampling();
        start_calculation();   
#if UART_SEND_DATA
        send_data();
#endif
        
        data_main->integ_value=integ_value;
        data_main->peak=peak;
        data_main->sampling_time=sampling_time;
      
        //MY_LOG( "\nsampling time:  " NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(data_main->integ_value));
        //MY_LOG( "\tpeak:  " NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(data_main->peak));
        //MY_LOG( "\tintegration:  " NRF_LOG_FLOAT_MARKER"\n ", NRF_LOG_FLOAT(data_main->sampling_time));
 
        

        memset(&data, 0, SIZE*sizeof(Accel_data_p_t));
        //return data_main;

}


/**@brief Function for finding out which achse was first triggered. 
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/
void MKI1178V1_get_triggered_axis()
{
   ret_code_t err_code = i2c_read2(WAKE_UP_SRC,&event_detection_status);
   APP_ERROR_CHECK(err_code);
   MY_LOG( "***  event_detection_status: x0%x  ***\n",event_detection_status);
   MY_LOG( "-------------------------------------------------------------------\n");
}


/**@brief Function for initializing the I2C protocol: The pin definition, the address, the operational frequency is defined. 
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/
void twi_init2 (void)
{

    NRF_TWIM0->PSEL.SCL = STEVAL_MKI1178V1_SCL_PIN;
    NRF_TWIM0->PSEL.SDA = STEVAL_MKI1178V1_SDA_PIN;

    NRF_TWIM0->ADDRESS = ADDRESS_MKI179_SAO_HIGH;
    NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400 << TWIM_FREQUENCY_FREQUENCY_Pos;
    NRF_TWIM0->SHORTS = 0;

    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
}


/**@brief initialize the I2C, reset the sensor and find out if we have the right sensor from the WHO_AM_I value
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/
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

/**@brief configures  the accelerometer and initializes the global array for handling the array accel values
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/
void MKI1178V1_conf(void)
{
  MKI1178V1_int();
  MKI1178V1_conf_interrupt();
  memset(&data, 0, SIZE*sizeof(Accel_data_p_t));
}


/**@brief writes the configuration for MKI1178V1 and enables the interrupt pins for the accelerometer.
 *        The offset, the interrupt source, sample rate, fifo configuration and treshold is defined.
 *
 * @param[out]  none      
 *
 * @param[in]   none  
 *
 * @return      none
 *
**/
void MKI1178V1_conf_interrupt()
{
  ret_code_t err_code;

  // empty the fifo buffer; it may happen the buffer is not empty from the previous measurment
   err_code = empty_the_fifo_Buffer();
   APP_ERROR_CHECK(err_code);

  // enable interrupt lines 
  nrfx_gpiote_in_event_enable(STEVAL_MKI1178V1_INT1_PIN, true);
  nrfx_gpiote_in_event_enable(STEVAL_MKI1178V1_INT2_PIN, true);


  
  err_code = i2c_write2(CTRL6,AMPLITUDE_RANGE_WITH_LOW_NOISE);  // define the amplitued value and enable low nosei
  APP_ERROR_CHECK(err_code);
  //i2c_read(CTRL6,&data);


  err_code = i2c_write2(CTRL7, CTRL7_INTERRUTPS_ENABLE | CTRL7_USR_OFF_W | CTRL7_USR_OFF_ON_OUT);  // interrupt enable | add a user offset weight | enable the output with the offset
  APP_ERROR_CHECK(err_code);
  //i2c_read(CTRL7,&data);


  err_code = i2c_write2(X_OFS_USR, X_OFS_USR_VALUE); // adding offset to the X achse
  APP_ERROR_CHECK(err_code);
  //i2c_read(X_OFS_USR,&data);


  //err_code = i2c_write2(Y_OFS_USR, Y_OFS_USR_VALUE); // adding offset to the Y achse
  err_code = i2c_write2(Y_OFS_USR, Y2_OFS_USR_VALUE); // adding offset to the Y achse
  APP_ERROR_CHECK(err_code);
  //i2c_read(Y_OFS_USR,&data);

  //err_code = i2c_write2(Z_OFS_USR, Z_OFS_USR_VALUE); // adding offset to the Z achse
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


#if FIFO_ENABLE

  err_code = i2c_write2(CTRL5_INT_PAD_CRL, CTRL5_INT_PAD_CRL_VALUE_FIFO_THRESHOLD); // FIFO threshold interrupt is routed to INT2
  err_code = i2c_write2(FIFO_CTRL, FIFO_CTRL_START); // configure the bypas-to-continues mode | set threshold to 20 
#else
  err_code = i2c_write2(CTRL5_INT_PAD_CRL, CTRL5_INT_PAD_CRL_VALUE_DATA_READY);
  APP_ERROR_CHECK(err_code);
  //i2c_read(CTRL5_INT_PAD_CRL,&data);
#endif

  APP_ERROR_CHECK(err_code);
  //i2c_read(FIFO_CTRL,&data);

  /* possible value to configure the sample rate

   CTRL1_value = CTRL1_ODR_50Hz; 
   CTRL1_value = CTRL1_ODR_HIGH_PEROFRAMNCE_400Hz; 
   CTRL1_value = CTRL1_ODR_HIGH_PEROFRAMNCE_800Hz; 
   CTRL1_value = CTRL1_ODR_HIGH_PEROFRAMNCE_1600Hz; 
  */

  CTRL1_value = CTRL1_ODR_HIGH_PEROFRAMNCE_800Hz; 
  
  err_code = i2c_write2(CTRL1, CTRL1_value); // define the sampling rate and the mode 
  APP_ERROR_CHECK(err_code);
  //i2c_read(CTRL1, &data);
}

/**@brief reads the threshold value and returns it back in mg
 *
 * @param[out]       none      
 *
 * @param[in, out]   threshold_in_bits threshold value directly from the register, which will be saved
 *
 * @return           the trehsold value in mg
 *
**/

int MKI1178V1_read_treshold_in_mg(uint8_t* threshold_in_bits){
  i2c_read2(WAKE_UP_THS, threshold_in_bits);
  trigger_value = *threshold_in_bits;
  return (trigger_value*quant_of_reg_val);

}


/**@brief resets the accelerometer defined in the datasheet
 *
 * @param[out]       none      
 *
 * @param[in]        none
 *
 * @return           none
 *
**/
static void MKI1178V1_reset_accel(){
    i2c_write2(CTRL2, CTRL2_SOFT_RESET);   
    nrf_delay_ms(1);
    i2c_write2(CTRL2, CTRL2_BOOT);   
    nrf_delay_ms(20);

}



/**@brief empties the fifo buffer if any data is present
 *
 * @param[out]       none      
 *
 * @param[in]        none
 *
 * @return           none
 *
**/
ret_code_t empty_the_fifo_Buffer()
{
  ret_code_t err_code;
  err_code = i2c_read2(STATUS,&status);
  APP_ERROR_CHECK(err_code);

 // the arrangement of the data values x,y,z are tighted to the register WAKE_UP_SRC
  while ( (status & STATUS_DRDY) == STATUS_DRDY){ //look if data is ready
     
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

/**@brief Function for reading the accelation in x achse. If the mean value is enabled then also
 *        the mean value is substracted which is hardcoded.  
 *
 * @param[out]  none      
 *                         
 *                         
 * @param[in]   Value   the acceleration value in x achse
 *
 * @return      none
 */
static void Read_X_Data(float* Value){

    uint8_t temp[2] = {0,0};
    int16_t tmp16 = 0;

    // read the lower byte of the value
    ret_code_t err_code  = i2c_read2(OUT_X_L, &temp[0]);
    APP_ERROR_CHECK(err_code);
    
    // read the higher byte of the value
    err_code  = i2c_read2(OUT_X_H, &temp[1]); 
    APP_ERROR_CHECK(err_code);


    // assemble the both bits
    tmp16 = (int16_t) (temp[1] << 8) | (int16_t)temp[0];	

    int index = (AMPLITUDE_RANGE_WITH_LOW_NOISE >> 4 ) & 0x03; // the first 2 bits of the second Nibble is what we want

    // find out in which range we are and apply the formel to it
#if AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_16g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/4) * calculation_values[index][1];
#elif AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_8g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/2) * calculation_values[index][1];
#else
    *Value = (double)(tmp16) / 	((calculation_values[index][0])) * calculation_values[index][1];
#endif

// substract  mean value if enabled
#if MeanValueEnable == 1
  *Value = *Value - MeanValueX;
#endif
}


/**@brief Function for reading the accelation in y achse. If the mean value is enabled then also
 *        the mean value is substracted which is hardcoded.  
 *
 * @param[out]  none      
 *                         
 *                         
 * @param[in]   Value   the acceleration value in y achse
 *
 * @return      none
 */
static void Read_Y_Data(float* Value){

    uint8_t temp[2] = {0,0};
    int16_t tmp16 = 0;

    // read out the lower byte of the result
    ret_code_t err_code  = i2c_read2(OUT_Y_L, &temp[0]);
    APP_ERROR_CHECK(err_code);

    // read out the higher byte of the result
    err_code  = i2c_read2(OUT_Y_H, &temp[1]); 
    APP_ERROR_CHECK(err_code);

   // assemble the result
    tmp16 = (int16_t) (temp[1] << 8) | (int16_t)temp[0];	

    int index = (AMPLITUDE_RANGE_WITH_LOW_NOISE >> 4 ) & 0x03; // the first 2 bits of the second Nibble is what we want

    // calculate now the real value
#if AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_16g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/4) * calculation_values[index][1];
#elif AMPLITUDE_RANGE_WITH_LOW_NOISE == (CTRL6_FS_8g | 0x04)
    *Value = (double)(tmp16) / 	((calculation_values[index][0])/2) * calculation_values[index][1];
#else
    *Value = (double)(tmp16) / 	((calculation_values[index][0])) * calculation_values[index][1];
#endif


#if MeanValueEnable == 1
  *Value = *Value - MeanValueY;
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

/**@brief Function for reading the accelation in z achse. If the mean value is enabled then also
 *        the mean value is substracted which is hardcoded.  
 *
 * @param[out]  none      
 *                         
 *                         
 * @param[in]   Value   the acceleration value in z achse
 *
 * @return      none
 */
static void Read_Z_Data(float* Value){

    uint8_t temp[2] = {0,0};
    int16_t tmp16 = 0;

    // read the lower byte of the Z achse
    ret_code_t err_code  = i2c_read2(OUT_Z_L, &temp[0]);
    APP_ERROR_CHECK(err_code);

    // read the higerh byte of the Z achse
    err_code  = i2c_read2(OUT_Z_H, &temp[1]); 
    APP_ERROR_CHECK(err_code);

    // assemble the data
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
     //MY_LOG( "*Value: %d\n",*Value);

#if MeanValueEnable == 1
  *Value = *Value - MeanValueZ;
#endif
}


/**@brief Function for writes the i2c messages
 *
 * @param[out]  none      
 *                         
 *                         
 * @param[in]   addr   the address of the register where to write
 *
 * @param[in]   data   the data which will be written to the register
 *
 * @return     NRF_SUCCESS when everything goes all well. The code will be stuck in the while
 *                         loop if the wires of the pins are not good attached. 
 */
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
 * @param[out]  none      
 *                         
 *                         
 * @param[in]   addr   the address of the register where to read
 *
 * @param[in]   data   the data which will be read from the register
 *
 * @return     NRF_SUCCESS when a value is received.
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


/**@brief Function for converting the force range from mg to the register value. 
 *        It converts the mg in quants per bit depending on the range_in_g 
 *
 * @param[out]  none      
 *                         
 *                         
 * @param[in]   range_in_g   in which range is the measurment attended 
 *
 * @param[in]   value_in_mg   the value in mg defined for the treshold
 *
 * @return      returns the range in in byte corresponding to the FR of the register 
 */
static uint8_t transfor_threshold_value_for_the_register(int range_in_g,int value_in_mg)
{
  // we are only interested in the last nibble, because there are the range configuration stored
  int bit_value_for_force_range =  range_in_g & 0xF0;
  int Register_Value = 0;


  switch(bit_value_for_force_range){
    case CTRL6_FS_2g:
      Register_Value = (value_in_mg+ QUANT_VAL_2g-1)/QUANT_VAL_2g;
      quant_of_reg_val = QUANT_VAL_2g;
      break;
    case CTRL6_FS_4g:
      //Register_Value = value_in_mg<<5 / CONVERT_G_TO_MG; // mutliplication by 32 and divide by 1000 to convert to mg
      Register_Value = (value_in_mg+QUANT_VAL_4g-1)/QUANT_VAL_4g;
      quant_of_reg_val = QUANT_VAL_4g;
      break;
    case CTRL6_FS_8g:
      Register_Value = (value_in_mg+QUANT_VAL_8g-1)/QUANT_VAL_8g;
      quant_of_reg_val = QUANT_VAL_8g;

#if DEBUG_LOG
#if MY_RTT    
      MY_LOG("Threshold value: %d\n", Register_Value);
#else
    NRF_LOG_RAW_INFO("Threshold value: %d\n", Register_Value);
#endif
#endif

      break;
    case CTRL6_FS_16g:
      Register_Value = (value_in_mg+QUANT_VAL_16g-1)/QUANT_VAL_16g;
      quant_of_reg_val = QUANT_VAL_16g;
      break;
  }

    return (uint8_t)Register_Value;
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