#include <Arduino.h>
// #include "imu_i.h"
#include <robo_Balance.h>
// #include <my_mpu.h>
// #include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include "transit_send_i.h"
// #include "trans_i.h"
// #include <HardwareSerial.h>
#include "servo_ctrl.h"


bool test = 0;
float phi_1_ini = 2.0893;
float phi_4_ini = 2.6510-1.57;

void task1(void *pvParameters) {
  while (1) {
    my_mpu_run();
    
  }
}

void task2(void *pvParameters) {

  while (1) {
    balance_run();
  }
}

void task3(void *pvParameters) {

  while (1) {
    
    balance_ctrl();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // vTaskDelay(pdMS_TO_TICKS(100));


  }
}

void setup(){
  Serial.begin(115200);
  Serial2.begin(9600);
  servo_init();
  my_mpu_init();
  balance_init();
  // imu_init();
  // trans_init();
  servo_ctrl(phi_1_ini, phi_4_ini);
  // setupBluetooth();
  delay(1000);
  xTaskCreatePinnedToCore(
        task1,   // Task function
        "imu",        // Task name
        4096,          // Stack size
        NULL,           // Task parameters
        1,              // Priority
        NULL,           // Task handle
        0               // Core number (0 for core 0, 1 for core 1)
  );
  xTaskCreatePinnedToCore(
        task2,   // Task function
        "balance",        // Task name
        4096,          // Stack size
        NULL,           // Task parameters
        1,              // Priority
        NULL,           // Task handle
        1               // Core number (0 for core 0, 1 for core 1)
  );
  xTaskCreatePinnedToCore(
        task3,   // Task function
        "trans",        // Task name
        5000,          // Stack size
        NULL,           // Task parameters
        2,              // Priority
        NULL,           // Task handle
        1               // Core number (0 for core 0, 1 for core 1)
  );

}


void loop(){
  // my_mpu_run();
  // trans_run();
  // my_mpu_run();
  // balance_run();
  // btrun_test();
  // static float a = 10.00;
  // // trans(a);
  // // a+=0.01;
  // // if(a>50) a = 0;
  // Serial2.print(a);
  // delay(1);
  // static float dataToSend = 1.17;
  // byte byteArray[sizeof(float)];
  // memcpy(byteArray, &dataToSend, sizeof(float));
  // Serial2.write(byteArray, sizeof(float));
  // dataToSend += 0.01;
  // if(dataToSend > 100){
  //   dataToSend = 0.53;
  // }
  // delay(20000);
  //     trans(20.0);
  //     delay(5000);
  // if(!test){
  //   trans(0.0);
  //   test = 1;
  // }
}



