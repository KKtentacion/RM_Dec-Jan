#ifndef __DRV_AS5600_H
#define __DRV_AS5600_H

#include "i2c.h"

#define AS5600_I2C_HANDLE hi2c2

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

/*
??:AS5600???0x36??????7?????,?ST I2C??????????????????????????
*/

#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)


#define AS5600_RESOLUTION 4096 //12bit Resolution 

#define AS5600_RAW_ANGLE_REGISTER  0x0C


void drv_as5600Init(void);
uint16_t drv_as5600GetRawAngle(void);
float drv_as5600GetAngle(void);

#endif /* __drv_AS5600_H */


