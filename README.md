# ShockDetectionBLEapplication

The whole project is divided into parts: master and peripheral controller, where the master is the receiver of the Bluetooth data and the peripheral controler generates the data. The peripheral controler gets the data from the accelerometer. The accelerometer measures a shock detection impact. The first data received from the shock detection impact is treated as raw data.  
From the raw data the duration, peak and energy of the impact is extrachted, where it is send over Bluetooth. 
