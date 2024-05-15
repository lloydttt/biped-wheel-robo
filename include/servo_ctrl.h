#ifndef SERVO_CTRL_H
#define SERVO_CTRL_H
#include <Arduino.h>

void servo_init();

float DataInChange(float k);

void LA_ctrl(float a);
void LB_ctrl(float a);
void RA_ctrl(float a);
void RB_ctrl(float a);

void servo_ctrl(float phi_1, float phi_4);

#endif

#define RA_PWM 2
#define RB_PWM 15
#define LA_PWM 33
#define LB_PWM 32
#define PI 3.1415926

int freq = 50;        // 频率
int resolution0 = 10; // 分辨率，取值0~20，duty最大取值为2^resolution-1
int channel0 = 0;     // 通道0，共16个通道，0~15
int channel1 = 1;     //
int channel2 = 2;     //
int channel3 = 3;     //

void servo_init(){
  pinMode(13, OUTPUT);
  ledcSetup(channel0, freq, resolution0); // 设置通道0
  ledcSetup(channel1, freq, resolution0); //
  ledcSetup(channel2, freq, resolution0); //
  ledcSetup(channel3, freq, resolution0); //
  // ledcSetup(channel4, freq, resolution0); //
  // ledcSetup(channel5, freq, resolution0); //

  ledcAttachPin(RA_PWM, channel0); // 将通道0与引脚13连接
  ledcAttachPin(RB_PWM, channel1); //
  ledcAttachPin(LA_PWM, channel2); //
  ledcAttachPin(LB_PWM, channel3); //
}

float DataInChange(float k){
    float angle_k = k/PI*180;
    // float a = 180;
    // float a = 0;
    float b = 0.5+(angle_k/180.0)*2;
    float c = (b/20.0)*1024-1;
    return c;
}

void LA_ctrl(float a){

  ledcWrite(channel2, a); // B3

}

void LB_ctrl(float a){
  ledcWrite(channel3, a); // B3

}


void RA_ctrl(float a){
  ledcWrite(channel0, a); // B3

}

void RB_ctrl(float a){
  ledcWrite(channel1, a); // B3

}


void servo_ctrl(float phi_1, float phi_4){
    phi_1 = DataInChange(phi_1);
    phi_4 = DataInChange(phi_4);
    LA_ctrl(phi_1);
    LB_ctrl(phi_4);
    RA_ctrl(phi_1);
    RB_ctrl(phi_4);
}


