#include <Arduino.h>
// #include "imu_i.h"
// #include <my_mpu.h>
// #include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include "transit_send_i.h"
#include "trans_i.h"

void setup(){
  Serial.begin(115200);
  // imu_init();
  // trans_init();
  // my_mpu_init();
  setupBluetooth();



}


void loop(){
  // imu_run();
  // trans_run();
  // my_mpu_run();
  // btrun_test();
  static float a = 10;
  trans(a);
  // a+=0.01;
  // if(a>50) a = 0;

  delay(1000);
}



