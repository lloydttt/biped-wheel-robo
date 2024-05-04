#ifndef ROBO_BALANCE_H
#define ROBO_BALANCE_H

//imu文件  提供位姿数据
#include <my_mpu.h>
#include "pid_defi.h"
#include "trans_i.h"

typedef struct{
    float vocility;
    bool orientation;
}Order_out;

// initialization
void balance_init();


void data_refresh();
// algorithm
// i 传出指令 ：前进or后退， 速度大小
void balance_run();


#endif