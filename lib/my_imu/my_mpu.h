#ifndef MY_MPU_H
#define MY_MPU_H
#pragma once

void my_mpu_init();

void my_mpu_run();

void data_transit(float &picth, float &roll, float &yaw);


#endif