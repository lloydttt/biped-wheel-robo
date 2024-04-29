#include "robo_Balance.h"

PID Angle_PID;
PID Speed_PID;
float Angle_Target = 0;
float Vocility_Target = 0;
float now_A, now_V;
float p_A = 0, i_A = 0, d_A = 0, Omax_A = 0, Imax_A = 0;
float p_V = 0, i_V = 0, d_V = 0, Omax_V = 0, Imax_V = 0;
float newAngle, newVocility;
float A_Target = 0, V_target = 0;

void balance_init(){
    PID_Init(&Angle_PID, p_A, i_A, d_A, Omax_A, Imax_A);
    PID_Init(&Speed_PID, p_V, i_V, d_V, Omax_V, Imax_V);

}

void data_refresh(){
    // 更新角度、速度数据。

}

void balance_run(){
    data_refresh();
    PID_calc(&Angle_PID, Angle_Target, newAngle);
    // PID_calc(&Speed_PID, Vocility_Target, newVocility);
}

// main.cpp中启用两个任务，mpu_run()与balance_run()，








