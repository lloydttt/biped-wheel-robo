#ifndef MY_MPU_H
#define MY_MPU_H

void setupBluetooth();
void my_mpu_init();

void my_mpu_run();

void data_transit(float &picth, float &roll, float &yaw);

void btrun_test();
#endif