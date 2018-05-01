/** @file MPU6050_DMP6.h
* 
* @brief Header Library to retreive pitch, roll values and setup IMU.
*
* @par   
*/ 

#ifndef _MPU6050_DMP6_H
#define _MPU6050_DMP6_H

extern float get_pitch();
extern float get_roll();
extern void imu_init();

#endif /* _MPU6050_DMP6_H */
