#include "robo_Balance.h"

PID Pitch_Angle_PID;
PID Roll_Angle_PID;
PID Speed_PID;
float Pitch_Angle_Target = 0;
float Roll_Angle_Target = 0;
float Vocility_Target = 0;
float now_A, now_V;
float p_A = 0, i_A = 0, d_A = 0, Omax_A = 0, Imax_A = 0;
float p_V = 0, i_V = 0, d_V = 0, Omax_V = 0, Imax_V = 0;
float P_newAngle, R_newAngle, Y_newAngle ,newVocility;
float A_Target = 0, V_target = 0;

void balance_init(){
    PID_Init(&Pitch_Angle_PID, p_A, i_A, d_A, Omax_A, Imax_A);
    PID_Init(&Speed_PID, p_V, i_V, d_V, Omax_V, Imax_V);

}

void data_refresh(){
    // 更新角度、速度数据。
    data_transit(P_newAngle, R_newAngle, Y_newAngle);
}

void balance_run(){
    data_refresh();
    PID_calc(&Pitch_Angle_PID, Pitch_Angle_Target, P_newAngle);
    trans(Pitch_Angle_PID.output);
    // PID_calc(&Speed_PID, Vocility_Target, newVocility);
}

// main.cpp中启用两个任务，mpu_run()与balance_run()，








