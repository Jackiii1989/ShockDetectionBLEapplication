# Abbreviations

BLE - bluetooth low energy

# Brief Description <h1> 
  Added are two folders -  project_for_the_master_central - and - project_for_the_master_peripheral - which represent two endpoints of the Bluetooth application. The project_for_the_master_peripheral detects shock impulse, basically values that reach a certion threshold of force or acceleration. project_for_the_master_central waits for the data and displayes it. 
  
 
  
  **project_for_the_master_peripheral:**
  
  The application advertises or broadcasts his data over BLE to get a connection and also at the same waits for the accelerometer to send data over i2C. With the first i2c package a timer is started. When the time period is over, the accelerometer goes into low-power mode and the peak, sampling time and integreation values are extracted/calculated. The parameters are send over UART, if enabled, and over BLE to the other endpoints as raw bytes.
  
  **project_for_the_master_central:**
  
  The central looks for other bluetooth device, where it connections and discovers the values that tje other end-point is holding. Sends couple of request to enable notifications for certain values and waits for the data to come. When the raw bytes are send over BLE, it then transforms to floats according to IEEE-11073 32-bitFLOAT format. (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa nd an 8-bit exponent.)
  
  1. Bluetooth communication files
   * ./project_for_the_maste_central/ble_cus_central.h
   * ./project_for_the_maste_central/ble_cus_central.c
   * ./project_for_the_master_peripheral/ble_cus.h
   * ./project_for_the_master_peripheral/ble_cus.c
   
 1. Accelerometer handling files
   * ./project_for_the_master_peripheral/steval_MKI178V1_central.c
   * ./project_for_the_master_peripheral/steval_MKI1178V1_central.h
