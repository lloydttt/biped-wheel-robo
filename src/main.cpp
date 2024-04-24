#include <Arduino.h>
// #include "imu_i.h"
// #include <my_mpu.h>
// #include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "transit_send_i.h"


void setup(){
  Serial.begin(115200);
  trans_init();



}


void loop(){
    // my_mpu_run();
  trans_run();





  
}



