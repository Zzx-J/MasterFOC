#include "bsp_as5600.h"


#define abs(x) ((x)>0?(x):-(x))
//#define _2PI 6.28318530718
#define _2PI 4096
extern Motor M1;
extern Motor M2;


void bsp_as5600Init(void) {
  /* init i2c interface */
  
  /* init var */
  //full_rotation_offset = 0;
  //angle_data_prev = bsp_as5600GetRawAngle();
}

static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count, Motor* mHandle) {
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  
  status = HAL_I2C_Master_Transmit(&mHandle->hI2CEncoder, dev_addr, pData, count, i2c_time_out);
  return status;
}

static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count, Motor* mHandle) {
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  
  status = HAL_I2C_Master_Receive(&mHandle->hI2CEncoder, (dev_addr | 1), pData, count, i2c_time_out);
  return status;
}

uint16_t bsp_as5600GetRawAngle(Motor* mHandle) {
  uint16_t raw_angle;
  uint8_t buffer[2] = {0};
  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
  
  i2cWrite(AS5600_ADDR, &raw_angle_register, 1, mHandle);
  i2cRead(AS5600_ADDR, buffer, 2, mHandle);
  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return raw_angle;
}

uint16_t bsp_as5600GetAngle(Motor* mHandle) {
  uint16_t angle;
  uint8_t buffer[2] = {0};
  uint8_t angle_register = AS5600_ANGLE_REGISTER;
  
  i2cWrite(AS5600_ADDR, &angle_register, 1, mHandle);
  i2cRead(AS5600_ADDR, buffer, 2, mHandle);
  angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
	mHandle->M_angle = angle;
  return angle;
}
