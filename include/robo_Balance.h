#ifndef ROBO_BALANCE_H
#define ROBO_BALANCE_H

//imu文件  提供位姿数据
#include <my_mpu.h>
#include "trans_i.h"
#include "pid_defi.h"
// #include "bttest.h"


typedef struct{
    float vocility;
    bool orientation;
}Order_out;

// initialization
void balance_init();


void data_refresh();

void balance_run();

void balance_ctrl();

// void ttt();
#endif

PID Pitch_Angle_PID;
PID Roll_Angle_PID;
PID Speed_PID;
float Pitch_Angle_Target = 0;
float Roll_Angle_Target = 0;
float Vocility_Target = 0;
float now_A, now_V;
float p_A = 0.75, i_A = 0, d_A = 0, Omax_A = 40, Imax_A = 0;
float p_V = 0, i_V = 0, d_V = 0, Omax_V = 20, Imax_V = 0;
float P_newAngle, R_newAngle, Y_newAngle ,newVocility;
float A_Target = -7.75, V_target = 0;

void balance_init(){
    PID_Init(&Pitch_Angle_PID, p_A, i_A, d_A, Omax_A, Imax_A);
    PID_Init(&Speed_PID, p_V, i_V, d_V, Omax_V, Imax_V);
    // if test
    // BT_init();
}

void data_refresh(){
    // 更新角度、速度数据、pid参数
    data_transit(P_newAngle, R_newAngle, Y_newAngle);

}

void balance_run(){
    data_refresh();
    PID_calc(&Pitch_Angle_PID, Pitch_Angle_Target, P_newAngle);
    // Serial.print(P_newAngle);
    // Serial.print(',');

    // trans(Pitch_Angle_PID.output);
    // PID_calc(&Speed_PID, Vocility_Target, newVocility);
}

void balance_ctrl(){
    trans(Pitch_Angle_PID.output);
    // Serial.println(Pitch_Angle_PID.output);

}

// main.cpp中启用两个任务，mpu_run()与balance_run()，


// void ttt(){
//         // if test
//     if(BT_testrun() == 'u'){
//         p_A += 0.1;
//         BT_back(p_A);
//     }else if(BT_testrun() == 'd'){
//         p_A -= 0.1;
//         BT_back(p_A);

//     }
// }