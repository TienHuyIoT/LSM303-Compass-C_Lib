/*
U2_PRINTF("\r\nAccelerometer:");
U2_PRINTF("\r\n X = %.4f Y = %.4f Z = %.4f",LSM303C_readAccelX(),LSM303C_readAccelY(),LSM303C_readAccelZ());
U2_PRINTF("\r\nMagnetometer:");
U2_PRINTF("\r\n X = %.4f Y = %.4f Z = %.4f\r\n",CVector_Read.xAxis,CVector_Read.yAxis,CVector_Read.zAxis);

Doc khi co calib
AxesRaw_t CVector_Read;
CVector_Read.xAxis -= ((int32_t)Mag_min.xAxis + Mag_max.xAxis) / 2;
CVector_Read.yAxis -= ((int32_t)Mag_min.yAxis + Mag_max.yAxis) / 2;
CVector_Read.zAxis -= ((int32_t)Mag_min.zAxis + Mag_max.zAxis) / 2;
U2_PRINTF("\r\n X = %d Y = %d Z = %d\r\n",CVector_Read.xAxis,CVector_Read.yAxis,CVector_Read.zAxis);

Doc nhiet do
U2_PRINTF("\r\nThermometer:");
U2_PRINTF("\r\n Degrees C = %.4f Degrees F = %.4f",LSM303C_readTempC(),LSM303C_readTempF());
*/

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "Wire.h"
#include "uart.h"
#include "SparkFunLSM303C.h"
// Define a few error messages to save on space
#define AERROR "Accel Error"
#define MERROR "Mag Error"
#define LSM303_Dbg(fmt,...) 	//(Poutput = UART2_OUTPUT,printf("\r\n>LSM303 " fmt, ##__VA_ARGS__))
#define IMU_PI   3.14159265358979323
// Variables to store the most recently read raw data from sensor
static AxesRaw_t accelData = {0,0,0};
static AxesRaw_t   magData = {0,0,0};
//Calib min: {  -239,   -306,   +382}    max: { +1017,   +994,  +1550}
//AxesRaw_t Mag_min = {  -239,   -306,   +382};
//AxesRaw_t Mag_max = {  +1017,   +994,  +1550};
//min: {  -711,  -1650,  -3399}    max: {  +591,   -406,  -2146}
AxesRaw_t Mag_min = {  -711,  -1650,  -3399};
AxesRaw_t Mag_max = {  +591,   -406,  -2146};

static void vector_normalize(Vector_t *a);
static void vector_cross(const Vector_t *a, const Vector_t *b, Vector_t *out);
static float vector_dot(Vector_t *a, Vector_t *b);

// The LSM303C functions over both I2C or SPI. This library supports both.
// Interface mode used must be set!
static InterfaceMode_t interfaceMode = MODE_I2C;  // Set a default...

static uint8_t  I2C_ByteWrite(I2C_ADDR_t, uint8_t, uint8_t);  
static status_t I2C_ByteRead(I2C_ADDR_t, uint8_t, uint8_t*);

// Methods required to get device up and running
static status_t MAG_SetODR(MAG_DO_t);
static status_t MAG_SetFullScale(MAG_FS_t);
static status_t MAG_BlockDataUpdate(MAG_BDU_t);
static status_t MAG_XY_AxOperativeMode(MAG_OMXY_t);
static status_t MAG_Z_AxOperativeMode(MAG_OMZ_t);
static status_t MAG_SetMode(MAG_MD_t);
static status_t ACC_SetFullScale(ACC_FS_t);
static status_t ACC_BlockDataUpdate(ACC_BDU_t);
static status_t ACC_EnableAxis(uint8_t);
static status_t ACC_SetODR(ACC_ODR_t);

static status_t ACC_Status_Flags(uint8_t*);
static status_t ACC_GetAccRaw(AxesRaw_t*);

static status_t MAG_GetMagRaw(AxesRaw_t*);
static status_t MAG_TemperatureEN(MAG_TEMP_EN_t);    
static status_t MAG_XYZ_AxDataAvailable(MAG_XYZDA_t*);
static float    readMag(AXIS_t);   // Reads the magnetometer data from IC

static status_t MAG_ReadReg(MAG_REG_t, uint8_t*);
static uint8_t  MAG_WriteReg(MAG_REG_t, uint8_t);
static status_t ACC_ReadReg(ACC_REG_t, uint8_t*);
static uint8_t  ACC_WriteReg(ACC_REG_t, uint8_t);

// Public methods
status_t LSM303C_begin_Default(void)
{
  return LSM303C_begin(// Default to I2C bus
        MODE_I2C,
        // Initialize magnetometer output data rate to 0.625 Hz (turn on device)
        MAG_DO_40_Hz,
        // Initialize magnetic field full scale to +/-16 gauss
        MAG_FS_16_Ga,
        // Enabling block data updating
        MAG_BDU_ENABLE,
        // Initialize magnetometer X/Y axes ouput data rate to high-perf mode
        MAG_OMXY_HIGH_PERFORMANCE,
        // Initialize magnetometer Z axis performance mode
        MAG_OMZ_HIGH_PERFORMANCE,
        // Initialize magnetometer run mode. Also enables I2C (bit 7 = 0)
        MAG_MD_CONTINUOUS,
        // Initialize acceleration full scale to +/-2g
        ACC_FS_2g,
        // Enable block data updating
        ACC_BDU_ENABLE,
        // Enable X, Y, and Z accelerometer axes
        ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE,
        // Initialize accelerometer output data rate to 100 Hz (turn on device)
        ACC_ODR_100_Hz
        );
}

status_t LSM303C_begin(InterfaceMode_t im, MAG_DO_t modr, MAG_FS_t mfs,
    MAG_BDU_t mbu, MAG_OMXY_t mxyodr, MAG_OMZ_t mzodr, MAG_MD_t mm,
    ACC_FS_t afs, ACC_BDU_t abu, uint8_t aea, ACC_ODR_t aodr)
{
  uint8_t successes = 0;
	interfaceMode = im;
// Enable I2C
  I2C0_Begin();
  ////////// Initialize Magnetometer //////////
  // Initialize magnetometer output data rate
  successes += MAG_SetODR(modr);
  // Initialize magnetic field full scale
  successes += MAG_SetFullScale(mfs);
  // Enabling block data updating
  successes += MAG_BlockDataUpdate(mbu);
  // Initialize magnetometer X/Y axes ouput data rate
  successes += MAG_XY_AxOperativeMode(mxyodr);
  // Initialize magnetometer Z axis performance mode
  successes += MAG_Z_AxOperativeMode(mzodr);
  // Initialize magnetometer run mode.
  successes += MAG_SetMode(mm);

  ////////// Initialize Accelerometer //////////
  // Initialize acceleration full scale
  successes += ACC_SetFullScale(afs);
  // Enable block data updating
  successes += ACC_BlockDataUpdate(abu);
  // Enable X, Y, and Z accelerometer axes
  successes += ACC_EnableAxis(aea);
  // Initialize accelerometer output data rate
  successes += ACC_SetODR(aodr);

  return (successes == IMU_SUCCESS) ? IMU_SUCCESS : IMU_HW_ERROR;
}

float LSM303C_readMagX()
{
  return readMag(xAxis);
}

float LSM303C_readMagY()
{
  return readMag(yAxis);
}

float LSM303C_readMagZ()
{
  return readMag(zAxis);
}

float LSM303C_readAccelX()
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = ACC_Status_Flags(&flag_ACC_STATUS_FLAGS);
  
  if (response != IMU_SUCCESS)
  {
    LSM303_Dbg(AERROR);
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_X_NEW_DATA_AVAILABLE)
  {
    uint8_t valueL;
    uint8_t valueH;

    if ( ACC_ReadReg(ACC_OUT_X_H, &valueH) )
    {
	    return IMU_HW_ERROR;
    }
  
    if ( ACC_ReadReg(ACC_OUT_X_L, &valueL) )
    {
	    return IMU_HW_ERROR;
    }
  
    LSM303_Dbg("Fresh raw data");

    //convert from LSB to mg
    return ((int16_t)( (valueH << 8) | valueL ) * SENSITIVITY_ACC);
  }

  // Should never get here
  LSM303_Dbg("Returning NAN");
  return NAN;
}

float LSM303C_readAccelY()
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = ACC_Status_Flags(&flag_ACC_STATUS_FLAGS);
  
  if (response != IMU_SUCCESS)
  {
    LSM303_Dbg(AERROR);
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_Y_NEW_DATA_AVAILABLE)
  {
    uint8_t valueL;
    uint8_t valueH;

    if ( ACC_ReadReg(ACC_OUT_Y_H, &valueH) )
    {
	    return IMU_HW_ERROR;
    }
  
    if ( ACC_ReadReg(ACC_OUT_Y_L, &valueL) )
    {
	    return IMU_HW_ERROR;
    }
  
    LSM303_Dbg("Fresh raw data");

    //convert from LSB to mg
    return ((int16_t)( (valueH << 8) | valueL ) * SENSITIVITY_ACC);
  }

  // Should never get here
  LSM303_Dbg("Returning NAN");
  return NAN;
}

float LSM303C_readAccelZ()
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = ACC_Status_Flags(&flag_ACC_STATUS_FLAGS);
  
  if (response != IMU_SUCCESS)
  {
    LSM303_Dbg(AERROR);
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_Z_NEW_DATA_AVAILABLE)
  {
    uint8_t valueL;
    uint8_t valueH;

    if ( ACC_ReadReg(ACC_OUT_Z_H, &valueH) )
    {
	    return IMU_HW_ERROR;
    }
  
    if ( ACC_ReadReg(ACC_OUT_Z_L, &valueL) )
    {
	    return IMU_HW_ERROR;
    }
  
    LSM303_Dbg("Fresh raw data");

    //convert from LSB to mg
    return((int16_t)( (valueH << 8) | valueL ) * SENSITIVITY_ACC);
  }

  // Should never get here
  LSM303_Dbg("Returning NAN");
  return NAN;
}


float LSM303C_readTempC()
{
  uint8_t valueL;
  uint8_t valueH;
  float temperature;

  // Make sure temperature sensor is enabled
  if( MAG_TemperatureEN(MAG_TEMP_EN_ENABLE))
  {
    return NAN;
  }

	if( MAG_ReadReg(MAG_TEMP_OUT_L, &valueL) )
  {
    return NAN;
  }

  if( MAG_ReadReg(MAG_TEMP_OUT_H, &valueH) )
  {
    return NAN;
  }

  temperature = (float)( (valueH << 8) | valueL );
  temperature /= 8; // 8 digits/˚C
  temperature += 25;// Reads 0 @ 25˚C

  return temperature;  
}

float LSM303C_readTempF()
{
  return( (LSM303C_readTempC() * 9.0 / 5.0) + 32.0);
}



////////////////////////////////////////////////////////////////////////////////
////// Protected methods

float LSM303C_readAccel(AXIS_t dir)
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = ACC_Status_Flags(&flag_ACC_STATUS_FLAGS);
  
  if (response != IMU_SUCCESS)
  {
    LSM303_Dbg(AERROR);
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_ZYX_NEW_DATA_AVAILABLE)
  {
    response = ACC_GetAccRaw(&accelData);
    LSM303_Dbg("Fresh raw data");
  }
  //convert from LSB to mg
  switch (dir)
  {
  case xAxis:
    return accelData.xAxis * SENSITIVITY_ACC;
    break;
  case yAxis:
    return accelData.yAxis * SENSITIVITY_ACC;
    break;
  case zAxis:
    return accelData.zAxis * SENSITIVITY_ACC;
    break;
  default:
    return NAN;
  }

  // Should never get here
  LSM303_Dbg("Returning NAN");
  return NAN;
}

float readMag(AXIS_t dir)
{
  MAG_XYZDA_t flag_MAG_XYZDA;
  status_t response = MAG_XYZ_AxDataAvailable(&flag_MAG_XYZDA);
  
  if (response != IMU_SUCCESS)
  {
    LSM303_Dbg(MERROR);
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  if (flag_MAG_XYZDA & MAG_XYZDA_YES)
  {
    response = MAG_GetMagRaw(&magData);
    LSM303_Dbg("Fresh raw data");
  }
  //convert from LSB to Gauss
  switch (dir)
  {
  case xAxis:
    return magData.xAxis * SENSITIVITY_MAG;
    break;
  case yAxis:
    return magData.yAxis * SENSITIVITY_MAG;
    break;
  case zAxis:
    return magData.zAxis * SENSITIVITY_MAG;
    break;
  default:
    return NAN;
  }

  // Should never get here
  LSM303_Dbg("Returning NAN");
  return NAN;
}

status_t MAG_GetMagRaw(AxesRaw_t* buff)
{
  uint8_t valueL;
  uint8_t valueH;
  
  LSM303_Dbg("& was false");
  if( MAG_ReadReg(MAG_OUTX_L, &valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( MAG_ReadReg(MAG_OUTX_H, &valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff->xAxis = (int16_t)( (valueH << 8) | valueL );
  
  if( MAG_ReadReg(MAG_OUTY_L, &valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( MAG_ReadReg(MAG_OUTY_H, &valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff->yAxis = (int16_t)( (valueH << 8) | valueL );
  
  if( MAG_ReadReg(MAG_OUTZ_L, &valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( MAG_ReadReg(MAG_OUTZ_H, &valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff->zAxis = (int16_t)( (valueH << 8) | valueL );

  return IMU_SUCCESS;
}

// Methods required to get device up and running
status_t MAG_SetODR(MAG_DO_t val)
{
  uint8_t value;

  if(MAG_ReadReg(MAG_CTRL_REG1, &value))
  {
    LSM303_Dbg("Failed Read from MAG_CTRL_REG1");
    return IMU_HW_ERROR;
  }

  // Mask and only change DO0 bits (4:2) of MAG_CTRL_REG1
  value &= ~MAG_DO_80_Hz;
  value |= val;

  if(MAG_WriteReg(MAG_CTRL_REG1, value))
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t MAG_SetFullScale(MAG_FS_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG2, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_FS_16_Ga; //mask
  value |= val;	

  if ( MAG_WriteReg(MAG_CTRL_REG2, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t MAG_BlockDataUpdate(MAG_BDU_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG5, &value) )
  {
    return IMU_HW_ERROR;
  }


  value &= ~MAG_BDU_ENABLE; //mask
  value |= val;		

  if ( MAG_WriteReg(MAG_CTRL_REG5, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t MAG_XYZ_AxDataAvailable(MAG_XYZDA_t* value)
{
  if ( MAG_ReadReg(MAG_STATUS_REG, (uint8_t*)value) )
  {
    return IMU_HW_ERROR;
  }

  *value = (MAG_XYZDA_t)((uint8_t)(*value) & (uint8_t)MAG_XYZDA_YES);

  return IMU_SUCCESS;
}

status_t MAG_XY_AxOperativeMode(MAG_OMXY_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG1, &value) )
  {
    return IMU_HW_ERROR;
  }
	
  value &= ~MAG_OMXY_ULTRA_HIGH_PERFORMANCE; //mask
  value |= val;	

  if ( MAG_WriteReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t MAG_Z_AxOperativeMode(MAG_OMZ_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG4, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_OMZ_ULTRA_HIGH_PERFORMANCE; //mask
  value |= val;	

  if ( MAG_WriteReg(MAG_CTRL_REG4, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t MAG_SetMode(MAG_MD_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG3, &value) )
  {
    LSM303_Dbg("Failed to read MAG_CTRL_REG3. 'Read': 0x%02X",value);
    return IMU_HW_ERROR;
  }

  value &= ~MAG_MD_POWER_DOWN_2;
  value |= val;		

  if ( MAG_WriteReg(MAG_CTRL_REG3, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t ACC_SetFullScale(ACC_FS_t val)
{
  uint8_t value;

  if ( ACC_ReadReg(ACC_CTRL4, &value) )
  {
    LSM303_Dbg("Failed ACC read");
    return IMU_HW_ERROR;
  }

  value &= ~ACC_FS_8g;
  value |= val;	


  if ( ACC_WriteReg(ACC_CTRL4, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t ACC_BlockDataUpdate(ACC_BDU_t val)
{
  uint8_t value;

  if ( ACC_ReadReg(ACC_CTRL1, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~ACC_BDU_ENABLE;
  value |= val;	

  if ( ACC_WriteReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t ACC_EnableAxis(uint8_t val)
{
  uint8_t value;

  if ( ACC_ReadReg(ACC_CTRL1, &value) )
  {
    LSM303_Dbg(AERROR);
    return IMU_HW_ERROR;
  }

  value &= ~0x07;
  value |= val;	

  if ( ACC_WriteReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t ACC_SetODR(ACC_ODR_t val)
{
  uint8_t value;

  if ( ACC_ReadReg(ACC_CTRL1, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~ACC_ODR_MASK;
  value |= val;	
	
  if ( ACC_WriteReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t MAG_TemperatureEN(MAG_TEMP_EN_t val){
  uint8_t value;

  if( MAG_ReadReg(MAG_CTRL_REG1, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_TEMP_EN_ENABLE; //mask
  value |= val;	

  if( MAG_WriteReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t MAG_ReadReg(MAG_REG_t reg, uint8_t* data)
{
	status_t ret = IMU_GENERIC_ERROR;

	ret = I2C_ByteRead(MAG_I2C_ADDR, reg, data);
	return ret;
}

uint8_t  MAG_WriteReg(MAG_REG_t reg, uint8_t data)
{
	uint8_t ret;
	ret = I2C_ByteWrite(MAG_I2C_ADDR, reg, data);
	return ret;
}

status_t ACC_ReadReg(ACC_REG_t reg, uint8_t* data)
{
	status_t ret;
	ret = I2C_ByteRead(ACC_I2C_ADDR, reg, data);
	return ret;
}

uint8_t  ACC_WriteReg(ACC_REG_t reg, uint8_t data)
{
	uint8_t ret;   
	ret = I2C_ByteWrite(ACC_I2C_ADDR, reg, data);
	return ret;
}

uint8_t  I2C_ByteWrite(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t data)
{
	uint8_t ret = IMU_GENERIC_ERROR;
	I2C_TransferReturn_TypeDef Rs;
	uint8_t WBuf[2] = {reg, data};
	Rs = I2C0_Write(slaveAddress,WBuf,2);
	if(Rs == i2cTransferDone){
		//LSM303_Dbg("write OK");
		ret = IMU_SUCCESS;
	}else{
		//LSM303_Dbg("write Err: %u",(uint8_t)Rs);
		ret = IMU_HW_ERROR;
	}
	return ret; 
}

status_t I2C_ByteRead(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t* data)
{
	status_t ret = IMU_GENERIC_ERROR;
	I2C_TransferReturn_TypeDef Rs;
	Rs = I2C0_WriteRead(slaveAddress,&reg,1,data,1);
	if(Rs == i2cTransferDone){
		//LSM303_Dbg("read OK");
		ret = IMU_SUCCESS;
	}else{
		//LSM303_Dbg("read Err: %u",Rs);
		ret = IMU_HW_ERROR;
	}	
	return ret; 
}

status_t ACC_Status_Flags(uint8_t* val)
{
  //LSM303_Dbg("Getting accel status");
  if( ACC_ReadReg(ACC_STATUS, val) )
  {
    LSM303_Dbg(AERROR);
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t ACC_GetAccRaw(AxesRaw_t* buff)
{
  uint8_t valueL;
  uint8_t valueH;

  if ( ACC_ReadReg(ACC_OUT_X_H, &valueH) )
  {
	  return IMU_HW_ERROR;
  }
  
  if ( ACC_ReadReg(ACC_OUT_X_L, &valueL) )
  {
	  return IMU_HW_ERROR;
  }
  
  buff->xAxis = (int16_t)( (valueH << 8) | valueL );
  
  if ( ACC_ReadReg(ACC_OUT_Y_H, &valueH) )
  {
	  return IMU_HW_ERROR;
  }
  
  if ( ACC_ReadReg(ACC_OUT_Y_L, &valueL) )
  {
	  return IMU_HW_ERROR;
  }
  
  buff->yAxis = (int16_t)( (valueH << 8) | valueL );
  
  if ( ACC_ReadReg(ACC_OUT_Z_H, &valueH) )
  {
	  return IMU_HW_ERROR;
  }
  
  if ( ACC_ReadReg(ACC_OUT_Z_L, &valueL) )
  {
	  return IMU_HW_ERROR;
  }

  buff->zAxis = (int16_t)( (valueH << 8) | valueL );

  return IMU_SUCCESS;
}

void LSM303_MCalib(void){
	static AxesRaw_t m_Min = {32767, 32767, 32767};
	static AxesRaw_t m_Max = {-32768, -32768, -32768};
	static AxesRaw_t m_Read;
	m_Read.xAxis = LSM303C_readMagX();
	m_Read.yAxis = LSM303C_readMagY();
	m_Read.zAxis = LSM303C_readMagZ();

	m_Min.xAxis = fmin(m_Min.xAxis, m_Read.xAxis);
	m_Min.yAxis = fmin(m_Min.yAxis, m_Read.yAxis);
	m_Min.zAxis = fmin(m_Min.zAxis, m_Read.zAxis);

	m_Max.xAxis = fmax(m_Max.xAxis, m_Read.xAxis);
	m_Max.yAxis = fmax(m_Max.yAxis, m_Read.yAxis);
	m_Max.zAxis = fmax(m_Max.zAxis, m_Read.zAxis);
	U2_PRINTF("\r\nMagnetometer:");
	U2_PRINTF("min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
			m_Min.xAxis,
			m_Min.yAxis,
			m_Min.zAxis,
			m_Max.xAxis,
			m_Max.yAxis,
			m_Max.zAxis);
}

float LSM303_Mag_Angular(void){
	Vector_t a,m;
	a.x = LSM303C_readAccelX();
	a.y = LSM303C_readAccelY();
	a.z = LSM303C_readAccelZ();
	m.x = LSM303C_readMagX();
	m.y = LSM303C_readMagY();
	m.z = LSM303C_readMagZ();
	return LSM303_heading(a,m);
}

/*Vector_t a,m;
a.x = LSM303C_readAccelX();
a.y = LSM303C_readAccelY();
a.z = LSM303C_readAccelZ();
m.x = LSM303C_readMagX();
m.y = LSM303C_readMagY();
m.z = LSM303C_readMagZ();
float heading = LSM303_heading(a,m);
U2_PRINTF("\r\nAngular: %.2f",heading);
 * */
float LSM303_heading(Vector_t a, Vector_t m)
{
	Vector_t temp_m = {m.x, m.y, m.z};

    // subtract offset (average of min and max) from magnetometer readings
    temp_m.x -= ((int32_t)Mag_min.xAxis + Mag_max.xAxis) / 2;
    temp_m.y -= ((int32_t)Mag_min.yAxis + Mag_max.yAxis) / 2;
    temp_m.z -= ((int32_t)Mag_min.zAxis + Mag_max.zAxis) / 2;

    // compute E and N
    Vector_t E;
    Vector_t N;
    Vector_t from = {1,0,0};
    vector_cross(&temp_m, &a, &E);
    vector_normalize(&E);
    vector_cross(&a, &E, &N);
    vector_normalize(&N);

    // compute heading IMU_PI
    float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / IMU_PI;
    if (heading < 0) heading += 360;
    // Swap Angle
    heading = 360 - heading;
    return heading;
}

void vector_normalize(Vector_t *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

void vector_cross(const Vector_t *a, const Vector_t *b, Vector_t *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

float vector_dot(Vector_t *a, Vector_t *b){
 return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}
