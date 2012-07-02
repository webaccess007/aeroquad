#define __AVR_ATmega2560__
#define __cplusplus
#define __builtin_va_list int
#define __attribute__(x)
#define __inline__
#define __asm__(x)
extern "C" void __cxa_pure_virtual() {}
#include "C:\Program Files\arduino-1.0\libraries\EEPROM\EEPROM.h"
#include "C:\Program Files\arduino-1.0\libraries\Wire\Wire.h"
#include "C:\Program Files\arduino-1.0\libraries\Wire\utility\twi.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Defines\GlobalDefined.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Defines\SensorsStatus.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Math\AQMath.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Math\FourtOrderFilter.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_BatteryMonitor\BatteryMonitor.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_BatteryMonitor\BatteryMonitorTypes.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gyroscope\Gyroscope.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gyroscope\Gyroscope_APM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gyroscope\Gyroscope_CHR6DM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gyroscope\Gyroscope_IDG_IDZ500.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gyroscope\Gyroscope_ITG3200.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gyroscope\Gyroscope_ITG3200_9DOF.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gyroscope\Gyroscope_ITG3200Common.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gyroscope\Gyroscope_Wii.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Accelerometer\Accelerometer.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Accelerometer\Accelerometer_ADXL345.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Accelerometer\Accelerometer_ADXL345_9DOF.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Accelerometer\Accelerometer_ADXL500.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Accelerometer\Accelerometer_APM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Accelerometer\Accelerometer_BMA180.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Accelerometer\Accelerometer_CHR6DM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Accelerometer\Accelerometer_WII.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_I2C\Device_I2C.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Platform_APM\APM_ADC.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Platform_APM\APM_ADC_Optimized.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Platform_APM\APM_RC.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Platform_Wii\Platform_Wii.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Platform_CHR6DM\Platform_CHR6DM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Kinematics\Kinematics.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Kinematics\Kinematics_ARG.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Kinematics\Kinematics_CHR6DM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Kinematics\Kinematics_DCM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Kinematics\Kinematics_MARG.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Compass\Compass.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Compass\Magnetometer_CHR6DM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Compass\Magnetometer_HMC5843.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Compass\Magnetometer_HMC5883L.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Compass\Magnetometer_HMC58xx.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Receiver\Receiver.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Receiver\Receiver_328p.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Receiver\Receiver_APM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Receiver\Receiver_HWPPM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Receiver\Receiver_MEGA.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Receiver\Receiver_PPM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Receiver\Receiver_RemotePC.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Motors\Motors.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Motors\Motors_APM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Motors\Motors_I2C.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Motors\Motors_PWM.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Motors\Motors_PWM_Timer.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Motors\Motors_TRI.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_BarometricSensor\BarometricSensor.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_BarometricSensor\BarometricSensor_BMP085.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_RangeFinder\MaxSonarRangeFinder.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_RangeFinder\RangeFinder.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_CameraStabilizer\CameraStabilizer.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_CameraStabilizer\CameraStabilizer_Aeroquad.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_CameraStabilizer\CameraStabilizer_Pins_2_3_5.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_CameraStabilizer\CameraStabilizer_Pins_44_45_46.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_CameraStabilizer\CameraStabilizer_Pins_6_7_8.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlHexPlus.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlHexX.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlHexY6.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlOctoPlus.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlOctoX.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlOctoX8.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlQuadPlus.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlQuadX.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlQuadY4.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlTri.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_FlightControlProcessor\FlightControlVariable.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gps\TinyGPS.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Gps\TinyGPSWrapper.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_SPI\Device_SPI.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456_AI.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456_Altitude.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456_Base.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456_BattMonitor.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456_Config.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456_Heading.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456_Notify.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456_RSSI.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\MAX7456_Timer.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_OSD\OSD.h"
#include "C:\Program Files\arduino-1.0\libraries\EEPROM\EEPROM.cpp"
#include "C:\Program Files\arduino-1.0\libraries\Wire\Wire.cpp"
#include "C:\Program Files\arduino-1.0\libraries\Wire\utility\twi.c"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_Math\AQMath.cpp"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\libraries\AQ_I2C\Device_I2C.cpp"
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void initPlatform();
void measureCriticalSensors();
void setup();
void loop ();

#include "C:\Program Files\arduino-1.0\hardware\arduino\variants\mega\pins_arduino.h" 
#include "C:\Program Files\arduino-1.0\hardware\arduino\cores\arduino\Arduino.h"
#include "Z:\Ted On My Mac\GitHub\FlightSoftware\AeroQuad\AeroQuad.ino" 
