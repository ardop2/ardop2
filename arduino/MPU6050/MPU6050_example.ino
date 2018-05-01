#include "MPU6050_DMP6.h"

void setup(){
  imu_init();
}

void loop(){
  Serial.print(get_pitch());
  Serial.print("\t");
  Serial.println(get_roll());
}

