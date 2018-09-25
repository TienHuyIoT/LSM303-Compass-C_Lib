/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "Wire.h"
#include "Sensor.h"
#include "LSM303_U_C.h"
#include "uart.h"
#define LSM303_Dbg(fmt,...) 	(Poutput = UART2_OUTPUT,printf("\r\n>LSM303 " fmt, ##__VA_ARGS__))
//enabling this #define will enable the debug print blocks
#define LSM303_DEBUG

static uint8_t write8(uint8_t address, uint8_t reg, uint8_t value);
static uint8_t read8(uint8_t address, uint8_t reg);
static bool readArray(uint8_t address, uint8_t reg, uint8_t* Buff, uint8_t Lenght);

static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain

/***************************************************************************
 ACCELEROMETER
 ***************************************************************************/
lsm303AccelData LSM303_Accel_Unified_raw;   // Last read accelerometer data will be available here 
static int32_t LSM303_Accel_Unified_sensorID;
static void LSM303_Accel_Unified_read(void);

/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/
lsm303MagData   LSM303_Mag_Unified_raw;     // Last read magnetometer data will be available here
lsm303MagGain   magGain;
bool            autoRangeEnabled;
static int32_t  LSM303_Mag_Unified_sensorID;
static void LSM303_Mag_Unified_read(void);

/***************************************************************************
 DEPRECATED (NON UNIFIED) DRIVER (Adafruit_LSM303.c/h)
 ***************************************************************************/
lsm303AccelData accelData;    // Last read accelerometer data will be available here
lsm303MagData magData;        // Last read magnetometer data will be available here

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
extern volatile uint32_t msTicks;
static uint32_t millis(void){
	return msTicks;
}
/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
uint8_t write8(uint8_t address, uint8_t reg, uint8_t value)
{
	I2C_TransferReturn_TypeDef Rs;
	uint8_t WBuf[2] = {reg, value};
	Rs = I2C0_Write(address,WBuf,2);
	if(Rs == i2cTransferDone){
		LSM303_Dbg("write OK");
	}else{
		LSM303_Dbg("write Err: %u",(uint8_t)Rs);
	}
	return (uint8_t)Rs;
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
uint8_t read8(uint8_t address, uint8_t reg)
{
	I2C_TransferReturn_TypeDef Rs;
	uint8_t Data;
	Rs = I2C0_WriteRead(address,&reg,1,&Data,1);
	if(Rs == i2cTransferDone){
		LSM303_Dbg("read OK");
		return Data;
	}
	LSM303_Dbg("read Err: %u",Rs);
	return 0xff;
}

bool readArray(uint8_t address, uint8_t reg, uint8_t* Buff, uint8_t Lenght)
{
	I2C_TransferReturn_TypeDef Rs;
	Rs = I2C0_WriteRead(address,&reg,1,Buff,Lenght);
	if(Rs == i2cTransferDone){
		LSM303_Dbg("readArray[%u] OK",Lenght);
		return true;
	}
	LSM303_Dbg("readArray[%u] Err: %u",Lenght,Rs);
	return false;
}

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void LSM303_Accel_Unified_read(void)
{
	uint8_t Data[6];
	readArray((uint8_t)LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80,Data,6);
	uint8_t xlo = Data[0];
	uint8_t xhi = Data[1];
	uint8_t ylo = Data[2];
	uint8_t yhi = Data[3];
	uint8_t zlo = Data[4];
	uint8_t zhi = Data[5];

	// Shift values to create properly formed integer (low uint8_t first)
	LSM303_Accel_Unified_raw.x = (int16_t)(xlo | (xhi << 8)) >> 4;
	LSM303_Accel_Unified_raw.y = (int16_t)(ylo | (yhi << 8)) >> 4;
	LSM303_Accel_Unified_raw.z = (int16_t)(zlo | (zhi << 8)) >> 4;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool LSM303_Accel_Unified_begin(int32_t sensorID)
{
  LSM303_Accel_Unified_sensorID = sensorID;
  // Clear the raw accel data
  LSM303_Accel_Unified_raw.x = 0;
  LSM303_Accel_Unified_raw.y = 0;
  LSM303_Accel_Unified_raw.z = 0;
  // Enable I2C
  I2C0_Begin();

  // Enable the accelerometer (100Hz)
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

  // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
  // if we are connected or not
  uint8_t reg1_a = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
  if (reg1_a != 0x57)
  {
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool LSM303_Accel_Unified_getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  LSM303_Accel_Unified_read();

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = LSM303_Accel_Unified_sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = millis();
  event->acceleration.x = (float)LSM303_Accel_Unified_raw.x * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = (float)LSM303_Accel_Unified_raw.y * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = (float)LSM303_Accel_Unified_raw.z * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void LSM303_Accel_Unified_getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LSM303", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = LSM303_Accel_Unified_sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0F; // TBD
  sensor->min_value   = 0.0F; // TBD
  sensor->resolution  = 0.0F; // TBD
}

/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void LSM303_Mag_Unified_read(void)
{
  uint8_t Data[6];
	readArray((uint8_t)LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_OUT_X_H_M,Data,6);
	uint8_t xlo = Data[0];
	uint8_t xhi = Data[1];
	uint8_t ylo = Data[2];
	uint8_t yhi = Data[3];
	uint8_t zlo = Data[4];
	uint8_t zhi = Data[5];

  // Shift values to create properly formed integer (low uint8_t first)
  LSM303_Mag_Unified_raw.x = (int16_t)(xlo | ((int16_t)xhi << 8));
  LSM303_Mag_Unified_raw.y = (int16_t)(ylo | ((int16_t)yhi << 8));
  LSM303_Mag_Unified_raw.z = (int16_t)(zlo | ((int16_t)zhi << 8));
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool LSM303_Mag_Unified_begin(int32_t sensorID)
{
  LSM303_Mag_Unified_sensorID = sensorID;
  autoRangeEnabled = false;

  // Clear the raw mag data
  LSM303_Mag_Unified_raw.x = 0;
  LSM303_Mag_Unified_raw.y = 0;
  LSM303_Mag_Unified_raw.z = 0;
  // Enable I2C
  I2C0_Begin();

  // Enable the magnetometer
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

  // LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
  // the default value (0b00010000/0x10)
  uint8_t reg1_a = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M);
  if (reg1_a != 0x10)
  {
    return false;
  }

  // Set the gain to a known level
  LSM303_Mag_Unified_setMagGain(LSM303_MAGGAIN_1_3);

  return true;
}

/**************************************************************************/
/*!
    @brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void LSM303_Mag_Unified_enableAutoRange(bool enabled)
{
  autoRangeEnabled = enabled;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void LSM303_Mag_Unified_setMagGain(lsm303MagGain gain)
{
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t)gain);

  magGain = gain;

  switch(gain)
  {
    case LSM303_MAGGAIN_1_3:
      _lsm303Mag_Gauss_LSB_XY = 1100;
      _lsm303Mag_Gauss_LSB_Z  = 980;
      break;
    case LSM303_MAGGAIN_1_9:
      _lsm303Mag_Gauss_LSB_XY = 855;
      _lsm303Mag_Gauss_LSB_Z  = 760;
      break;
    case LSM303_MAGGAIN_2_5:
      _lsm303Mag_Gauss_LSB_XY = 670;
      _lsm303Mag_Gauss_LSB_Z  = 600;
      break;
    case LSM303_MAGGAIN_4_0:
      _lsm303Mag_Gauss_LSB_XY = 450;
      _lsm303Mag_Gauss_LSB_Z  = 400;
      break;
    case LSM303_MAGGAIN_4_7:
      _lsm303Mag_Gauss_LSB_XY = 400;
      _lsm303Mag_Gauss_LSB_Z  = 355;
      break;
    case LSM303_MAGGAIN_5_6:
      _lsm303Mag_Gauss_LSB_XY = 330;
      _lsm303Mag_Gauss_LSB_Z  = 295;
      break;
    case LSM303_MAGGAIN_8_1:
      _lsm303Mag_Gauss_LSB_XY = 230;
      _lsm303Mag_Gauss_LSB_Z  = 205;
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's update rate
*/
/**************************************************************************/
void LSM303_Mag_Unified_setMagRate(lsm303MagRate rate)
{
	uint8_t reg_m = ((uint8_t)rate & 0x07) << 2;
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, reg_m);
}


/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool LSM303_Mag_Unified_getEvent(sensors_event_t *event) {
  bool readingValid = false;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  while(!readingValid)
  {

    uint8_t reg_mg = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_SR_REG_Mg);
    if (!(reg_mg & 0x1)) {
			return false;
    }

    /* Read new data */
    LSM303_Mag_Unified_read();

    /* Make sure the sensor isn't saturating if auto-ranging is enabled */
    if (!autoRangeEnabled)
    {
      readingValid = true;
    }
    else
    {
#ifdef LSM303_DEBUG
	LSM303_Dbg("%u %u %u",LSM303_Mag_Unified_raw.x,LSM303_Mag_Unified_raw.y,LSM303_Mag_Unified_raw.z);
#endif
      /* Check if the sensor is saturating or not */
      if ( (LSM303_Mag_Unified_raw.x >= 2040) | (LSM303_Mag_Unified_raw.x <= -2040) |
           (LSM303_Mag_Unified_raw.y >= 2040) | (LSM303_Mag_Unified_raw.y <= -2040) |
           (LSM303_Mag_Unified_raw.z >= 2040) | (LSM303_Mag_Unified_raw.z <= -2040) )
      {
        /* Saturating .... increase the range if we can */
        switch(magGain)
        {
          case LSM303_MAGGAIN_5_6:
            LSM303_Mag_Unified_setMagGain(LSM303_MAGGAIN_8_1);
            readingValid = false;
#ifdef LSM303_DEBUG
            LSM303_Dbg("Changing range to +/- 8.1");
#endif
            break;
          case LSM303_MAGGAIN_4_7:
            LSM303_Mag_Unified_setMagGain(LSM303_MAGGAIN_5_6);
            readingValid = false;
#ifdef LSM303_DEBUG
            LSM303_Dbg("Changing range to +/- 5.6");
#endif
            break;
          case LSM303_MAGGAIN_4_0:
            LSM303_Mag_Unified_setMagGain(LSM303_MAGGAIN_4_7);
            readingValid = false;
#ifdef LSM303_DEBUG
            LSM303_Dbg("Changing range to +/- 4.7");
#endif
            break;
          case LSM303_MAGGAIN_2_5:
            LSM303_Mag_Unified_setMagGain(LSM303_MAGGAIN_4_0);
            readingValid = false;
#ifdef LSM303_DEBUG
            LSM303_Dbg("Changing range to +/- 4.0");
#endif
            break;
          case LSM303_MAGGAIN_1_9:
            LSM303_Mag_Unified_setMagGain(LSM303_MAGGAIN_2_5);
            readingValid = false;
#ifdef LSM303_DEBUG
            LSM303_Dbg("Changing range to +/- 2.5");
#endif
            break;
          case LSM303_MAGGAIN_1_3:
            LSM303_Mag_Unified_setMagGain(LSM303_MAGGAIN_1_9);
            readingValid = false;
#ifdef LSM303_DEBUG
            LSM303_Dbg("Changing range to +/- 1.9");
#endif
            break;
          default:
            readingValid = true;
            break;
        }
      }
      else
      {
        /* All values are withing range */
        readingValid = true;
      }
    }
  }

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = LSM303_Mag_Unified_sensorID;
  event->type      = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = millis();
  event->magnetic.x = (float)LSM303_Mag_Unified_raw.x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y = (float)LSM303_Mag_Unified_raw.y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z = (float)LSM303_Mag_Unified_raw.z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

	return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void LSM303_Mag_Unified_getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LSM303", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = LSM303_Mag_Unified_sensorID;
  sensor->type        = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0F; // TBD
  sensor->min_value   = 0.0F; // TBD
  sensor->resolution  = 0.0F; // TBD
}

/* --- The code below is no longer maintained and provided solely for */
/* --- compatibility reasons! */

/***************************************************************************
 DEPRECATED (NON UNIFIED) DRIVER (Adafruit_LSM303.c/h)
 ***************************************************************************/

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
bool Adafruit_LSM303_begin()
{
  I2C0_Begin();

  // Enable the accelerometer
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27);

  // Enable the magnetometer
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM303_read()
{
	uint8_t Data[6];
	readArray((uint8_t)LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80,Data,6);
	uint8_t xlo = Data[0];
	uint8_t xhi = Data[1];
	uint8_t ylo = Data[2];
	uint8_t yhi = Data[3];
	uint8_t zlo = Data[4];
	uint8_t zhi = Data[5];

	// Shift values to create properly formed integer (low uint8_t first)
	accelData.x = (int16_t)((uint16_t)xlo | ((uint16_t)xhi << 8)) >> 4;
	accelData.y = (int16_t)((uint16_t)ylo | ((uint16_t)yhi << 8)) >> 4;
	accelData.z = (int16_t)((uint16_t)zlo | ((uint16_t)zhi << 8)) >> 4;

	// Read the magnetometer

	readArray((uint8_t)LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_OUT_X_H_M,Data,6);
	xlo = Data[0];
	xhi = Data[1];
	ylo = Data[2];
	yhi = Data[3];
	zlo = Data[4];
	zhi = Data[5];

	// Shift values to create properly formed integer (low uint8_t first)
	magData.x = (int16_t)((uint16_t)xlo | ((uint16_t)xhi << 8));
	magData.y = (int16_t)((uint16_t)ylo | ((uint16_t)yhi << 8));
	magData.z = (int16_t)((uint16_t)zlo | ((uint16_t)zhi << 8));
}

void Adafruit_LSM303_setMagGain(lsm303MagGain gain)
{
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t)gain);
}
