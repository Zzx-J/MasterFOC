#ifndef __BSP_AS5600_H
#define __BSP_AS5600_H

#include "i2c.h"
#include "control.h"

#define AS5600_I2C_HANDLE1 hi2c2
#define AS5600_I2C_HANDLE2 hi2c1

#define I2C_TIME_OUT_BASE   5
#define I2C_TIME_OUT_BYTE   1

/*
注意:AS5600的地址0x36是指的是原始7位设备地址,而ST I2C库中的设备地址是指原始设备地址左移一位得到的设备地址
*/

#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)


#define AS5600_RESOLUTION 4096 //12bit Resolution 

#define AS5600_RAW_ANGLE_REGISTER  0x0C
#define AS5600_ANGLE_REGISTER  0x0E

void bsp_as5600Init(void);
uint16_t bsp_as5600GetRawAngle(Motor* mHandle);
uint16_t bsp_as5600GetAngle(Motor* mHandle);

#endif /* __BSP_AS5600_H */
