// Abstract class for SFE IMU products.  Provides interchangeable interface
// and serves as a template.

#ifndef __SPARKFUNIMU_H__
#define __SPARKFUNIMU_H__
// Return values 
typedef enum
{
  IMU_SUCCESS,
  IMU_HW_ERROR,
  IMU_NOT_SUPPORTED,
  IMU_GENERIC_ERROR,
  IMU_OUT_OF_BOUNDS,
  //...
} status_t;

#endif // End of __SPARKFUNIMU_H__ definition check
