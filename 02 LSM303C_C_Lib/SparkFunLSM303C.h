// Test derrived class for base class SparkFunIMU
#ifndef __SPARKFUN_LSM303C_H__
#define __SPARKFUN_LSM303C_H__
#include <stdint.h>
#include "SparkFunIMU.h"
#include "LSM303CTypes.h"

//#define SENSITIVITY_ACC   0.000061   // LSB/mg
//#define SENSITIVITY_MAG   0.00058   // LSB/Ga

#define SENSITIVITY_ACC   1   // LSB/mg
#define SENSITIVITY_MAG   1   // LSB/Ga

status_t LSM303C_begin_Default(void);
// Begin contains hardware specific code (Pro Mini)
status_t LSM303C_begin(InterfaceMode_t, MAG_DO_t, MAG_FS_t, MAG_BDU_t, MAG_OMXY_t,
   MAG_OMZ_t, MAG_MD_t, ACC_FS_t, ACC_BDU_t, uint8_t, ACC_ODR_t);
float LSM303C_readAccelX(void);
float LSM303C_readAccelY(void);
float LSM303C_readAccelZ(void);
float LSM303C_readMagX(void);
float LSM303C_readMagY(void);
float LSM303C_readMagZ(void);
float LSM303C_readTempC(void);
float LSM303C_readTempF(void);
float LSM303C_readAccel(AXIS_t); // Reads the accelerometer data from IC
void LSM303_MCalib(void);
float LSM303_heading(Vector_t a, Vector_t m);
float LSM303_Mag_Angular(void);
AxesRaw_t Mag_min;
AxesRaw_t Mag_max;

#endif
