

#ifndef STEVAL_MKI1178V1_H__
#define STEVAL_MKI1178V1_H__

#include "MacroDefinitions.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

#include "app_timer.h"


#include "app_uart.h"
#include "nrf_uart.h"
#include "nrf.h"

#include <nrf.h>

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

#include "boards.h"
#include "app_util_platform.h"

#include "ble_cus_central.h"

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------------------------*/
/* DEFINES  -------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------*/

/* Addres of the Accel --------------------------------------------------------*/

#define ADDRESS_MKI179_SAO_HIGH        				(0x32>>1)  // Real 7 bits slave address value in Datasheet is: b0001100x
#define ADDRESS_MKI179_SAO_LOW         				(0x24>>1)  // Real 7 bits slave address value in Datasheet is: b0001100x

/* Regsiter for Aceel --------------------------------------------------------*/
#define OUT_T_L               						0x0DU
#define OUT_T_H               						0x0EU
#define WHO_AM_I              						0x0FU
#define CTRL1                 						0x20U
#define CTRL2                 						0x21U
#define CTRL3                 						0x22U
#define CTRL4_INT_PAD_CRL     						0x23U
#define CTRL5_INT_PAD_CRL     						0x24U
#define CTRL6                 						0x25U
#define STATUS                						0x27U
#define OUT_X_L               						0x28U
#define OUT_X_H               						0x29U
#define OUT_Y_L               						0x2AU
#define OUT_Y_H               						0x2BU
#define OUT_Z_L               						0x2CU
#define OUT_Z_H               						0x2DU
#define FIFO_CTRL             						0x2EU
#define FIFO_SAMPLES          						0x2fU
#define WAKE_UP_THS           						0x34U
#define WAKE_UP_DUR           						0x35U
#define STATUS_DUP     	      						0x37U
#define X_OFS_USR     	      						0x3CU
#define WAKE_UP_SRC           						0x38U
#define Y_OFS_USR     	      						0x3DU
#define Z_OFS_USR     	      						0x3EU
#define CTRL7                 						0x3fU 


// Coded values for the register
#define STATUS_DRDY                					(0x01)
#define CTRL1_STOP_THE_SENSOR					(0x00)
#define CTRL1_HIGH_POWER_MODE              			(0x04)
#define CTRL1_SINGLE_MODE                  			(0x08)                                       // update rate 50 Hz and single data conversation on demand
#define CTRL1_ODR_1_6Hz	   		   		   	(0x10)
#define CTRL1_ODR_12_5Hz		   			(0x20)
#define CTRL1_ODR_50Hz                     			(0x40)
#define CTRL1_ODR_100Hz                    			(0x50)
#define CTRL1_ODR_200Hz                    			(0x60)
#define CTRL1_ODR_HIGH_PEROFRAMNCE_100Hz   			(CTRL1_ODR_100Hz | CTRL1_HIGH_POWER_MODE)
#define CTRL1_ODR_HIGH_PEROFRAMNCE_200Hz   			(CTRL1_ODR_200Hz | CTRL1_HIGH_POWER_MODE)
#define CTRL1_ODR_HIGH_PEROFRAMNCE_400Hz   			(0x70 | CTRL1_HIGH_POWER_MODE)
#define CTRL1_ODR_HIGH_PEROFRAMNCE_800Hz   			(0x80 | CTRL1_HIGH_POWER_MODE)
#define CTRL1_ODR_HIGH_PEROFRAMNCE_1600Hz  			(0x90 | CTRL1_HIGH_POWER_MODE)
#define CTRL2_SOFT_RESET							(0x40)										// all intern register of the accelerometer goes to 0							
#define CTRL2_BOOT									(0x80)										// boot the accelerometer: load the trimming values from nonvolatile memory 							
#define WHO_AM_I_ANSWER                   			(0x44)                                      //  defined value in register WHO_AM_I
#define CTRL4_INT_PAD_CRL_ROUTE_INT1_TO_WAKE_UP                 (0x20)										//where to route the interrupt INT1 line
#define CTRL5_INT_PAD_CRL_VALUE_FIFO_THRESHOLD                  (0x02)										//where to route the interrupt INT2 line
#define CTRL6_ENABLE_LOW_NOISE_CONF        			(0x04)										// low noise is activated
#define CTRL6_FS_2g       	           				(0x00) 										//  range -/+ 2g
#define CTRL6_FS_4g       	           				(0x01 << 4) 								//  range -/+ 4g
#define CTRL6_FS_8g       	           				(0x02 << 4) 								//  range -/+ 8g
#define CTRL6_FS_16g       		   					(0x03 << 4) 								//  range -/+ 16g
#define AMPLITUDE_RANGE_WITH_LOW_NOISE     			(CTRL6_FS_8g | CTRL6_ENABLE_LOW_NOISE_CONF) // -+16g with low noise enabled          
#define CTRL7_INTERRUTPS_ENABLE            			(0x20)                                       // enable interrupts on the pins INT1 and INT2
#define CTRL7_USR_OFF_ON_OUT               			(0x10)                                       // the defined offset will be also seen in the output registers
#define CTRL7_USR_OFF_W                    			(0x04)                                       // the defined offset values: not every bit corresponds to 16.6 mg/LSB; mg--> gravitational force
#define CTRL7_END_VALUE                    			(CTRL7_INTERRUTPS_ENABLE | CTRL7_USR_OFF_W | CTRL7_USR_OFF_ON_OUT)

#define FIFO_CTRL_MASK     							0x3F             						 	// this value is to get only the FIFO values
#define FIFO_CTRL_STOP     							(0 << 5) | (21)              				// first part to stop the fifo | threshold values in buffer how many it should be
#define FIFO_CTRL_START     						(4 << 5) | (21)              				// first part to stop the fifo | threshold values in buffer how many it should be
#define TRESHOLD_VALUE_MG                   		(125)  										// threshold value in mg
#define WAKE_UP_DUR_VALUE                   		(0x00)  									// threshold value for time duration: how long has to be threshold
																								// present for the interrupt to trigger



#define X_POSITION                          2
#define Y_POSITION							1
#define Z_POSITION							0
#define EXTRA_VALUES_TO_SEND				10
#define CONVERT_G_TO_MG						1000
#define X_OFS_USR_VALUE						0xFF
#define Y_OFS_USR_VALUE						0xFE
#define Z_OFS_USR_VALUE						0xC0

// pin definitions 
#define BUTTON_1                             		13
#define BUTTON_2                             		14
#define BUTTON_3                             		15
#define BUTTON_4                             		16

#define STEVAL_MKI1178V1_SCL_PIN             		27    // SCL signal pin
#define STEVAL_MKI1178V1_SDA_PIN             		26    // SDA signal pin

#define STEVAL_MKI1178V1_INT1_PIN            		20   //  interrupt pin 1 for the accel 
#define STEVAL_MKI1178V1_INT2_PIN            		19   //  interrupt pin 2 for the accel 

#define STEVAL_MKI1178V1_CS_PIN            		24   //  interrupt pin 1 for the accel 
#define STEVAL_MKI1178V1_SA0_PIN            		25   //  interrupt pin 2 for the accel 


/*-----------------------------------------------------------------------------*/
/* GLOBAL VALUES  -------------------------------------------------------------*/
/*-----------------------------------------------------------------------------*/
// parameters for calculus function 
static float max_value;
static float min_value;
static  float differ;
static volatile float peak;
static volatile float integ_value;
static volatile float sampling_time;
static float dt;
static size_t iter;



// typedefs
typedef enum
{
    ACCEL_EVT_TIMER_ENABLE,                             /**< Custom value notification enabled event. */
    ACCEL_EVT_TIMER_DISABLE,                             /**< Custom value notification disabled event. */
    ACCEL_EVT_UPDATE
} accel_evt_type_t;
typedef struct
{
    accel_evt_type_t evt_type;                                  /**< Type of event. */
} accel_evt_t;


typedef struct accel_s accel_t;

typedef void (*accel_evt_handler_t) (accel_t * p_accel_data, accel_evt_t* p_evt);


struct accel_s
{
   float peak;
   float integ_value;
   float sampling_time;
   
};

typedef struct 
{
  accel_evt_handler_t evt_handler_struct;
}accel_event;

// data point for the accelerometer 
typedef struct
{
  float values[3]; // 3 floats where 1. is z, 2. is y, 3. is x --> because of the wake_up_src
} Accel_data_p_t;

//

//--------------------------------------------------------------------------------------------------------------
 // this look-up table is defined after the bit values or ODR and MODE
//they are different sampling times depending on the energy mode
static double dt_values[][2] =  
	{ // low-power mode | High performance
		{0.0, 	       0.0},  	  // 0 Hz   , 0 Hz 
		{0.625,       0.08},  	// 1.6 Hz , 12.5 Hz 
		{0.08,        0.08},  	// 12.5 Hz , 12.5 Hz 
		{0.04,        0.04},    // 25 Hz   , 25 Hz
		{0.02,        0.02},    // 50 hz
		{0.01,        0.01},    // 100 hz
		{0.005,       0.005},    // 200 Hz
		{0.005,       0.0025},   // 200 Hz, 400 Hz
		{0.005,       0.00125},  // 200 Hz, 800 Hz
		{0.005,       0.000625}     // 200 Hz, 1600 Hz
	}; 
	
static double calculation_values[][2] =   // this look-up table is defined from the  FS, tables values in Datasheet and High-performance 
	{ // Full-scale selection | sensitivity 
		{2, 		    		 0.244},  	  // +-2g  ,  0.244 mg/digit
		{4, 		    		 0.488},  	  // +-4g  ,  0.488 mg/digit
		{8,             		 0.976},  	  // +-8g  ,  0.976 mg/digit
		{16,            		 1.952},      // +-16g ,  1.952 mg/digit
	}; 	


void MKI1178V1_save_data();
static void MKI1178V1_init();
void MKI1178V1_get_triggered_axis();
static void MKI1178V1_reset_accel();
static void MKI1178V1_uart_init();
static void MKI1178V1_conf_interrupt();


//void MKI1178V1_conf(accel_event evt_struct);
void MKI1178V1_conf(void);

void uart_error_handle(app_uart_evt_t * p_event);


void put_in_active_mode(void);
static void send_data(void);
static uint8_t transfor_threshold_value_for_the_register(int range_in_g,int value_in_mg);
static void stop_sampling(void);
static void start_calculation(void);
ret_code_t empty_the_fifo_Buffer();
accel_t processing();

static void Read_X_Data(float* Value);
static void Read_Y_Data(float* Value);
static void Read_Z_Data(float* Value);


static void accel_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);


static void lfclk_request(void);
void MKI1178V1_timer_init();
void MKI1178V1_timer_init_for_BLE();
static void notification_timeout_handler(void * p_context);


static void twi_init2 (void);
static ret_code_t i2c_write2(uint8_t addr, uint8_t data);
static ret_code_t i2c_read2(uint8_t addr, uint8_t* data);




#ifdef __cplusplus
}
#endif

#endif // STEVAL_MKI1178V1_H__